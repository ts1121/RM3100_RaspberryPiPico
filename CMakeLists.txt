cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rm3100_i2c_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(rm3100_i2c
	rm3100_i2c.c
)

pico_enable_stdio_usb(rm3100_i2c 1)
pico_enable_stdio_uart(rm3100_i2c 0)

pico_add_extra_outputs(rm3100_i2c)

target_link_libraries(rm3100_i2c pico_stdlib hardware_i2c)