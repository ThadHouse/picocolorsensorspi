# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_FETCH_FROM_GIT ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(picocolorsensorspi C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(picocolorsensorspi
        src/picocolorsensorspi.c
        src/core1.cpp
        src/pio_spi.c
        segger/SEGGER_RTT_printf.c
        segger/SEGGER_RTT.c)

target_include_directories(picocolorsensorspi PRIVATE segger)

pico_generate_pio_header(picocolorsensorspi ${CMAKE_CURRENT_LIST_DIR}/pio/pio_spi.pio)

pico_set_program_name(picocolorsensorspi "picocolorsensorspi")
pico_set_program_version(picocolorsensorspi "0.1")

pico_enable_stdio_uart(picocolorsensorspi 1)
pico_enable_stdio_usb(picocolorsensorspi 0)

# Add the standard library to the build
target_link_libraries(picocolorsensorspi pico_stdlib)

# Add any user requested libraries
target_link_libraries(picocolorsensorspi
        hardware_spi
        hardware_i2c
        hardware_pio
        hardware_dma
        pico_multicore
        )

pico_add_extra_outputs(picocolorsensorspi)

target_compile_options(picocolorsensorspi PRIVATE -Werror -Wall -Wextra)
