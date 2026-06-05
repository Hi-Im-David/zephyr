.. zephyr:code-sample:: code_relocation_nocopy_programmer
   :name: Code Relocation NoCopy External Flash Programmer

   Program the external MSPI NOR flash with the XIP payload produced by the
   :zephyr:code-sample:`code_relocation_nocopy` sample.

Overview
********

This sample is a one-shot on-device flasher used to install the
``.extflash_text_reloc`` bytes of the
:zephyr:code-sample:`code_relocation_nocopy` sample into external MSPI NOR
flash on boards where the host programmer (for example ``nrfutil``) cannot
write to the XIP aperture directly.

The program:

1. Reads an embedded payload (``src/payload.bin``), which is linked into the
   main image via ``.incbin``.
2. Opens the flash device referenced by the ``flash0`` devicetree alias.
3. Erases the first erase-block-aligned region at offset 0.
4. Writes the payload.
5. Reads the payload back and compares it.
6. Prints ``programmer: OK`` and exits.

Because the programmer writes to offset 0 of ``flash0``, it matches the
placement of ``.extflash_text_reloc`` at the start of the XIP aperture in
:file:`samples/application_development/code_relocation_nocopy/linker_arm_nocopy.ld`.

This keeps the consumer sample's build graph simple - it just links code at
the XIP aperture - and isolates the programming logic in a separate image.

End-to-end workflow
*******************

1. **Build the consumer sample** (produces the XIP payload at link time):

   .. code-block:: bash

      west build -b nrf7120dk/nrf7120/cpuapp \
          zephyr/samples/application_development/code_relocation_nocopy -p

2. **Extract the payload** from the linked ELF. The
   ``.extflash_text_reloc`` section holds the bytes that will live in
   external flash:

   .. code-block:: bash

      arm-zephyr-eabi-objcopy -O binary \
          --only-section=.extflash_text_reloc \
          build/zephyr/zephyr.elf \
          zephyr/samples/application_development/code_relocation_nocopy_programmer/src/payload.bin

   (The exact ``objcopy`` binary comes with your Zephyr SDK; adjust the
   path if your toolchain prefix differs.)

3. **Build and flash the programmer** (this sample):

   .. code-block:: bash

      west build -b nrf7120dk/nrf7120/cpuapp \
          zephyr/samples/application_development/code_relocation_nocopy_programmer -p
      west flash

   Expected UART output:

   .. code-block:: none

      programmer: starting
      programmer: payload size = <N> bytes
      programmer: erasing <M> bytes at offset 0 (erase block <K>)
      programmer: writing <N> bytes
      programmer: verifying
      programmer: OK (<N> bytes at offset 0 of <device>)

4. **Flash the consumer sample**. Its hex contains only MRAM content; the
   external flash has already been populated in step 3:

   .. code-block:: bash

      west build -b nrf7120dk/nrf7120/cpuapp \
          zephyr/samples/application_development/code_relocation_nocopy
      west flash

5. Reset. The consumer sample now executes ``function_in_ext_flash`` via
   XIP from the MSPI NOR flash.

Requirements
************

- A board with an MSPI NOR flash whose devicetree node exposes the
  ``flash0`` alias (see :file:`boards/nrf7120dk_nrf7120_cpuapp.overlay`).
- The MSPI driver and ``jedec,mspi-nor``/``jedec,spi-nor`` bindings
  enabled (see :file:`prj.conf`).

Adapting to other boards
************************

Add a ``boards/<your_board>.overlay`` enabling the MSPI controller and
flash chip, and setting ``aliases { flash0 = &<flash_node>; };``. The
programmer logic is board-agnostic.

Notes
*****

- Re-run ``objcopy`` whenever you change the consumer sample and the XIP
  payload bytes change; ``src/payload.bin`` is a build input here.
- The checked-in ``src/payload.bin`` is a placeholder pattern
  (``DE AD BE EF`` repeated) so the sample builds out of the box.
  Replace it with the real extracted payload before flashing real
  hardware.
