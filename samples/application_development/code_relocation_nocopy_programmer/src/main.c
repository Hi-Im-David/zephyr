/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * One-shot MSPI NOR programmer for the code_relocation_nocopy sample.
 *
 * At build time, src/payload.bin (the raw contents of the consumer sample's
 * .extflash_text_reloc section) is embedded into rodata via .incbin. At
 * boot, we erase the first erase-block-aligned region of flash0 and write
 * the payload there, verifying with a read-back. The destination is offset
 * 0 of flash0; the consumer sample's linker script places the XIP section
 * at the start of the external flash aperture, so offset 0 on the flash
 * device corresponds to the first instruction.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

LOG_MODULE_REGISTER(programmer, LOG_LEVEL_INF);

#define FLASH_NODE DT_ALIAS(flash0)
BUILD_ASSERT(DT_NODE_EXISTS(FLASH_NODE),
	     "DT alias \"flash0\" must point to an MSPI NOR flash");

/*
 * Embed the payload via a dedicated asm file-scope block. Using .incbin
 * from an __asm__ at file scope places the bytes in a read-only section
 * in MRAM, and gives us global start/end symbols we can reference from C.
 *
 * PAYLOAD_PATH is defined by CMake (target_compile_definitions) and
 * stringified here via the usual STR(x) trick so the preprocessor expands
 * the macro before the assembler sees it.
 */
#define STR_(x) #x
#define STR(x)  STR_(x)

__asm__ (
	".section .rodata.programmer_payload, \"a\", %progbits\n"
	".balign 4\n"
	".global programmer_payload_start\n"
	"programmer_payload_start:\n"
	".incbin " STR(PAYLOAD_PATH) "\n"
	".global programmer_payload_end\n"
	"programmer_payload_end:\n"
	".previous\n"
);

extern const uint8_t programmer_payload_start[];
extern const uint8_t programmer_payload_end[];

#define VERIFY_CHUNK 256

/*
 * The nrf7120 MSPI XIP read path byte-swaps each 32-bit word on the way to
 * the AHB so that a little-endian CPU sees the Thumb instruction stream in
 * its natural order. To satisfy that, the payload must be stored in flash
 * with each word byte-reversed. This helper takes the raw, CPU-order
 * payload and produces the flash-order image, padding any trailing sub-word
 * with 0xFF so the final word is also swapped coherently.
 */
static void swap_words_for_xip(uint8_t *dst, const uint8_t *src, size_t len)
{
	size_t full_words = len / 4;

	for (size_t i = 0; i < full_words; i++) {
		uint32_t w;

		memcpy(&w, src + i * 4, sizeof(w));
		w = __builtin_bswap32(w);
		memcpy(dst + i * 4, &w, sizeof(w));
	}

	size_t tail = len - full_words * 4;
	if (tail) {
		uint8_t pad[4] = { 0xff, 0xff, 0xff, 0xff };

		memcpy(pad, src + full_words * 4, tail);
		uint32_t w;

		memcpy(&w, pad, sizeof(w));
		w = __builtin_bswap32(w);
		memcpy(dst + full_words * 4, &w, sizeof(w));
	}
}

static int verify(const struct device *flash, const uint8_t *src, size_t len)
{
	uint8_t buf[VERIFY_CHUNK];

	for (size_t off = 0; off < len; off += sizeof(buf)) {
		size_t n = MIN(sizeof(buf), len - off);
		int rc = flash_read(flash, off, buf, n);

		if (rc) {
			LOG_ERR("flash_read(%zu) failed: %d", off, rc);
			return rc;
		}

		printk("flash[0x%08zx]:", off);
		for (size_t i = 0; i < n; i++) {
			printk(" %02x", buf[i]);
		}
		printk("\n");

		if (memcmp(buf, src + off, n) != 0) {
			LOG_ERR("verify mismatch at offset %zu", off);
			return -EIO;
		}
	}
	return 0;
}

/*
 * Staging buffer for the byte-swapped payload. Sized to match the sample's
 * current payload (56 bytes) with generous headroom; bump if the consumer
 * payload grows.
 */
#define PAYLOAD_STAGE_MAX 4096
static uint8_t payload_stage[PAYLOAD_STAGE_MAX] __aligned(4);

int main(void)
{
	const struct device *flash = DEVICE_DT_GET(FLASH_NODE);
	const size_t payload_size =
		(size_t)(programmer_payload_end - programmer_payload_start);
	const size_t payload_size_padded = ROUND_UP(payload_size, 4);
	struct flash_pages_info page;
	size_t erase_len;
	int rc;

	printk("programmer: starting\n");
	printk("programmer: payload size = %zu bytes\n", payload_size);

	if (!device_is_ready(flash)) {
		LOG_ERR("flash device %s not ready", flash->name);
		return -ENODEV;
	}

	if (payload_size == 0) {
		LOG_ERR("payload is empty; did you forget to replace src/payload.bin?");
		return -EINVAL;
	}

	if (payload_size_padded > sizeof(payload_stage)) {
		LOG_ERR("payload (%zu bytes, padded to %zu) exceeds stage buffer (%zu)",
			payload_size, payload_size_padded, sizeof(payload_stage));
		return -ENOMEM;
	}

	/*
	 * The MSPI XIP read path on nrf7120 byte-swaps each 32-bit word on
	 * the way to the AHB. To get a correct Thumb instruction stream at
	 * the CPU, the payload must be stored in flash with each word
	 * byte-reversed. Stage the swapped image in RAM and write/verify
	 * against that.
	 */
	swap_words_for_xip(payload_stage, programmer_payload_start, payload_size);

	rc = flash_get_page_info_by_offs(flash, 0, &page);
	if (rc) {
		LOG_ERR("flash_get_page_info_by_offs failed: %d", rc);
		return rc;
	}

	erase_len = ROUND_UP(payload_size_padded, page.size);
	printk("programmer: erasing %zu bytes at offset 0 (erase block %zu)\n",
	       erase_len, page.size);

	rc = flash_erase(flash, 0, erase_len);
	if (rc) {
		LOG_ERR("flash_erase failed: %d", rc);
		return rc;
	}

	printk("programmer: writing %zu bytes (byte-swapped for XIP)\n",
	       payload_size_padded);
	rc = flash_write(flash, 0, payload_stage, payload_size_padded);
	if (rc) {
		LOG_ERR("flash_write failed: %d", rc);
		return rc;
	}

	printk("programmer: verifying (swapped image)\n");
	rc = verify(flash, payload_stage, payload_size_padded);
	if (rc) {
		LOG_ERR("verify failed: %d", rc);
		return rc;
	}

	printk("programmer: OK (%zu bytes at offset 0 of %s)\n",
	       payload_size, flash->name);
	return 0;
}
