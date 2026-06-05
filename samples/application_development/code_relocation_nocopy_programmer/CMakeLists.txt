# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(code_relocation_nocopy_programmer)

target_sources(app PRIVATE src/main.c)

# Embed src/payload.bin into the image via .incbin. The asm in main.c uses
# the PAYLOAD_PATH macro so the absolute path survives out-of-tree builds.
target_compile_definitions(app PRIVATE
  PAYLOAD_PATH="${CMAKE_CURRENT_SOURCE_DIR}/src/payload.bin"
)

# Ensure a rebuild when the payload changes.
set_property(
  DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/payload.bin
)
