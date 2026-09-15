# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(blinky)

target_sources(app PRIVATE src/main.c src/boot_trace_dump.c)

# Reaches the MDK's system_nrf71.c as well as app code, which is where the
# capture side of the boot trace lives.
zephyr_compile_definitions(NRF_ENABLE_NRF7120_BOOT_TRACE)
