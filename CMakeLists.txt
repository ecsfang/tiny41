cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(tiny41 C CXX ASM)

set(PICO_COPY_TO_RAM 1)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(tiny41
  tiny41.cpp
  ssd1306.cpp
  core_bus.cpp
)

pico_enable_stdio_usb(tiny41 1)
pico_enable_stdio_uart(tiny41 0)

pico_add_extra_outputs(tiny41)

target_link_libraries(
  tiny41
  pico_stdlib
  hardware_i2c
  hardware_clocks
	hardware_adc
  pico_multicore
)