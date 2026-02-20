cmake_minimum_required(VERSION 3.13...3.27)

set(NAME RV4028-io)
set(PICO_PLATFORM rp2350-riscv)
set(PICO_BOARD pimoroni_pga2350)
set(PICO_TOOLCHAIN_PATH $ENV{PICO_SDK_PATH}/../riscv-toolchain)

set(PICO_COPY_TO_RAM 1)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_compile_options(-Wall -Werror
        -Wno-format
        -Wno-unused-function
        )

# Add your source files
add_executable(${NAME}
    main.c # <-- Add source files here!
)

target_link_libraries(${NAME}
    pico_stdlib
    hardware_dma
    hardware_pio
    hardware_pwm
    hardware_clocks
)

pico_generate_pio_header(${NAME} ${CMAKE_CURRENT_LIST_DIR}/bus.pio)
pico_generate_pio_header(${NAME} ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 1)

# Set up files for the release packages
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.uf2
    ${CMAKE_CURRENT_LIST_DIR}/README.md
    DESTINATION .
)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)
