cmake_minimum_required(VERSION 3.13..3.27)

include(pico_sdk_import.cmake)

project(tiny41 C CXX ASM)

set(PICO_COPY_TO_RAM 1)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(RP2350_PSRAM_CS 47)

pico_sdk_init()

add_executable(tiny41
  tiny41.cpp
  ssd1306.cpp
  core_bus.cpp
  disasm.cpp
  disstr.cpp
  serial.cpp
  ir_led.cpp
  modfile.cpp
  module.cpp
  xfmem.cpp
  flash.cpp
  wand.cpp
  timemod.cpp
  voyager.cpp
  usb/cdc_helper.cpp            # helper functions for the multi-CDC USB interface
  usb/usb-descriptors.cpp       # descriptors for the multi-CDC USB interface
)

add_subdirectory(/home/EU/sefangeth/Projects/rp2040-psram rp2040-psram)

# Create C header file with the name <pio program>.pio.h
pico_generate_pio_header(tiny41
  ../ir_pio/hp41_pio_irout.pio
)

#pico_set_linker_script(tiny41 ${CMAKE_SOURCE_DIR}/memmap_custom.ld)

pico_enable_stdio_usb(tiny41 0)
pico_enable_stdio_uart(tiny41 1)

pico_add_extra_outputs(tiny41)

target_compile_definitions(tiny41 PRIVATE
    # PSRAM_MUTEX=1
    PSRAM_SPINLOCK=1
    PSRAM_ASYNC=1
    PSRAM_PIN_CS=9
    PSRAM_PIN_SCK=10
    PSRAM_PIN_MOSI=11
    PSRAM_PIN_MISO=12
)

target_link_libraries(
  tiny41
  pico_stdlib
  pico_multicore
  pico_flash
  hardware_i2c
  hardware_clocks
	hardware_adc
  hardware_pio
  tinyusb_device
  hardware_flash
)

target_include_directories(tiny41 PRIVATE
  .
  $ENV{PICO_SDK_PATH}/lib/tinyusb/hw
  $ENV{PICO_SDK_PATH}/lib/tinyusb/src
)
