/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mspi_slave, LOG_LEVEL_DBG);

#define MSPI_PERIPHERAL_NODE 	DT_NODELABEL(peripheral)
#define MSPI_CONTROLLER_NODE 	DT_NODELABEL(controller)

#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);


#define DATA_LINES_MAX 4

#define SCK_FREQUENCY MHZ(1)

#define CMD_LEN 1
#define ADDR_LEN 3
#define DATA_LEN 32
#define NUM_PACKETS 2
#define TX_XFER_TYPE MSPI_DMA
#define RX_XFER_TYPE MSPI_DMA

static uint8_t packet_buf1[DATA_LEN];
static uint8_t packet_buf2[DATA_LEN];
static uint8_t packet_buf3[DATA_LEN];


static uint8_t rx_buff1[DATA_LEN + CMD_LEN + ADDR_LEN];
static uint8_t rx_buff2[DATA_LEN + CMD_LEN + ADDR_LEN];

static const struct device *mspi_slave 	= 	DEVICE_DT_GET(MSPI_PERIPHERAL_NODE);
static const struct device *mspi_master 	= 	DEVICE_DT_GET(MSPI_CONTROLLER_NODE);

static const struct mspi_dev_id tx_id = {
	.dev_idx = 0,
};

static const struct mspi_dev_id rx_id = {
	.dev_idx = 0,
};

void setup_buffer(void)
{
	// packet_buf1[0] = 0x00010203;
	// packet_buf1[1] = 0x04050607;
	// packet_buf1[2] = 0x08091011;
	// packet_buf1[3] = 0x12131415;

	for (int i = 0; i < DATA_LEN; ++i) {
		packet_buf1[i] = (uint8_t)i;
		packet_buf2[i] = (uint8_t)100+i;
		packet_buf3[i] = (uint8_t)7;
	}
	return;
}

void print_rx_buff(uint8_t * input_buff) {
    for (size_t i = 0; i < DATA_LEN + CMD_LEN + ADDR_LEN; i++) {

        printk("returned buffer [%u] = 0x%2x\n", i, input_buff[i]);
    }
}

void async_cb(struct mspi_callback_context *mspi_cb_ctx)
{
	// volatile struct user_context *usr_ctx = mspi_cb_ctx->ctx;
	printk("printing rx data....\r\n");

	// print_rx_buff(mspi_cb_ctx->mspi_evt.evt_data.packet->data_buf);
	print_rx_buff(rx_buff1);
	printk("\r\n\r\n");
	print_rx_buff(rx_buff2);
}

void master_async_cb(struct mspi_callback_context *mspi_cb_ctx)
{
	// volatile struct user_context *usr_ctx = mspi_cb_ctx->ctx;
	printk("reeeeeeeeeeeeee\r\n");

	print_rx_buff(mspi_cb_ctx->mspi_evt.evt_data.packet->data_buf);
}

struct user_context {
	uint32_t status;
	uint32_t total_packets;
};

int main()
{
	int ret;

	// if (!gpio_is_ready_dt(&led)) {
	// 	return 0;
	// }

	// ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	// if (ret < 0) {
	// 	return 0;
	// }

	// gpio_pin_toggle_dt(&led);
	// for(int i = 0; i < 99999; i++) {
	// 	__NOP();
	// }

	// // gpio_pin_toggle_dt(&led);
	// for(int i = 0; i < 99999; i++) {
	// 	__NOP();
	// }

	// // gpio_pin_toggle_dt(&led);
	// for(int i = 0; i < 99999; i++) {
	// 	__NOP();
	// }


	setup_buffer();
	if(!device_is_ready(mspi_slave))
	{
		LOG_ERR("MSPI device %s is not ready", mspi_slave->name);
	}

	if(!device_is_ready(mspi_master))
	{
		LOG_ERR("MSPI device %s is not ready", mspi_master->name);
	}

	struct mspi_dev_cfg rx_dev_cfg = {
		.ce_num = 1,
		.freq = SCK_FREQUENCY,
		.io_mode = MSPI_IO_MODE_QUAD_1_1_4,
		.data_rate = MSPI_DATA_RATE_SINGLE,
		.cpp = MSPI_CPP_MODE_0,
		.endian = MSPI_XFER_BIG_ENDIAN,
		.ce_polarity = MSPI_CE_ACTIVE_LOW,
		.cmd_length = CMD_LEN,
		.addr_length = ADDR_LEN,
	};
	struct mspi_xfer_packet rx_packet1 = {
		.dir = MSPI_RX,
		.cmd = 0,
		.address = 0,
		.data_buf = rx_buff1,
		.num_bytes = DATA_LEN + CMD_LEN + ADDR_LEN,
		.cb_mask = MSPI_BUS_XFER_COMPLETE_CB,
	};
	struct mspi_xfer_packet rx_packet2 = {
		.dir = MSPI_RX,
		.cmd = 0,
		.address = 0,
		.data_buf = rx_buff2,
		.num_bytes = DATA_LEN + CMD_LEN + ADDR_LEN,
		.cb_mask = MSPI_BUS_XFER_COMPLETE_CB,
	};

	struct mspi_xfer_packet rx_packets[] = {rx_packet1, rx_packet2};

	struct mspi_xfer rx_xfer = {
		.xfer_mode   = RX_XFER_TYPE,
		.packets     = rx_packets,
		.num_packet  = NUM_PACKETS,
		.timeout     = 500,
		.async		 = true,
		.cmd_length = CMD_LEN,
		.addr_length = ADDR_LEN,
	};

	struct mspi_dev_cfg tx_dev_cfg = {
		.ce_num = 1,
		.freq = SCK_FREQUENCY,
		.io_mode = MSPI_IO_MODE_QUAD_1_1_4,
		.data_rate = MSPI_DATA_RATE_SINGLE,
		.cpp = MSPI_CPP_MODE_0,
		.endian = MSPI_XFER_BIG_ENDIAN,
		.ce_polarity = MSPI_CE_ACTIVE_LOW,
		.cmd_length = CMD_LEN,
		.addr_length = ADDR_LEN,
	};

	struct mspi_xfer_packet tx_packet1 = {
		.dir = MSPI_TX,
		.cmd = 0x99999999,
		.address = 0x12345678,
		.data_buf = (uint8_t*)packet_buf1,
		.num_bytes = DATA_LEN,
	};

	struct mspi_xfer_packet tx_packet2 = {
		.dir = MSPI_TX,
		.cmd = 0x99999999,
		.address = 0x0,
		.data_buf = packet_buf2,
		.num_bytes = DATA_LEN,
		
	};
	struct mspi_xfer_packet tx_packet3 = {
		.dir = MSPI_TX,
		.cmd = 0x55,
		.address = 0x123456,
		.data_buf = packet_buf3,
		.num_bytes = DATA_LEN,
	};

	struct mspi_xfer_packet tx_packets[] = {tx_packet1, tx_packet2, tx_packet3};
	struct mspi_xfer tx_xfer = {
		.xfer_mode   = RX_XFER_TYPE,
		.packets     = tx_packets,
		.num_packet  = NUM_PACKETS,
		.timeout     = 500,
		.cmd_length = CMD_LEN,
		.addr_length = ADDR_LEN,
		// .async		 = true,
		.tx_dummy = 0,
		.rx_dummy = 0,
	};
	printk("hi\r\n");

	uint8_t cmd_lines, addr_lines, data_lines;
	cmd_lines = 4;
	addr_lines = 4;
	data_lines = 4;
	int rc;
	volatile struct user_context read_ctx;
	volatile struct user_context controller_ctx;

	uint8_t cmd_addr_cycles = (tx_xfer.cmd_length * 8 / cmd_lines)
			+ (tx_xfer.addr_length * 8 / addr_lines);
	tx_xfer.tx_dummy = 0;//8 - (cmd_addr_cycles % 8);

	read_ctx.total_packets  = rx_xfer.num_packet;
	read_ctx.status         = ~0;
	struct mspi_callback_context cb_ctx;
	cb_ctx.ctx = (void *)&read_ctx;

	struct mspi_callback_context controller_cb_ctx;
	controller_cb_ctx.ctx = (void *)&controller_ctx;

	rc = mspi_dev_config(mspi_slave, &rx_id,
			     MSPI_DEVICE_CONFIG_ALL, &rx_dev_cfg);
	if (rc) {
		printk("Failed to config peripheral\n");
	}
	rc = mspi_dev_config(mspi_master, &tx_id,
			     MSPI_DEVICE_CONFIG_ALL, &tx_dev_cfg);
	if (rc) {
		printk("Failed to config controller\n");
	}

	rc = mspi_register_callback(mspi_slave, &rx_id, MSPI_BUS_XFER_COMPLETE,
					(mspi_callback_handler_t)async_cb, &cb_ctx);
	if (rc) {
		printk("Failed to register callback\n");
	}

	// rc = mspi_register_callback(mspi_master, &tx_id, MSPI_BUS_XFER_COMPLETE,
	// 	(mspi_callback_handler_t)master_async_cb, &cb_ctx);
	// if (rc) {
	// printk("Failed to register callback\n");
	// }

	// while(1)
	// {
		printk("Setting Async rx...\r\n");
		rc = mspi_transceive(mspi_slave, &rx_id, &rx_xfer);
		if(rc != 0)
		{
			printk("returned: %d\r\n", rc);
		}


		k_msleep(5);
		printk("- 8-bit command, 24-bit address\n");
		
		rc = mspi_transceive(mspi_master, &tx_id, &tx_xfer);
		if(rc != 0)
		{
			printk("returned: %d\r\n", rc);
		}
		// k_msleep(1000);

		// printk("Setting Async rx...\r\n");
		// rc = mspi_transceive(mspi_slave, &rx_id, &rx_xfer);
		// if(rc != 0)
		// {
		// 	printk("returned: %d\r\n", rc);
		// }


		// k_msleep(5);
		// printk("- 8-bit command, 24-bit address\n");
		
		// rc = mspi_transceive(mspi_master, &tx_id, &tx_xfer);
		// if(rc != 0)
		// {
		// 	printk("returned: %d\r\n", rc);
		// }
		// k_msleep(1000);

		// printk("tx_xfer.address_length = %d\r\n", tx_xfer.addr_length);

	return 0;
}
