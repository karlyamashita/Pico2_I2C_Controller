# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2_w CACHE STRING "Board type" FORCE)

include(pico_sdk_import.cmake)

project(Pico2W_I2C_Controller C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(Pico2W_I2C_Controller
    main.c
)

pico_generate_pio_header(Pico2W_I2C_Controller ${CMAKE_CURRENT_LIST_DIR}/i2c_sniff.pio)
pico_generate_pio_header(Pico2W_I2C_Controller ${CMAKE_CURRENT_LIST_DIR}/spi_sniff.pio)

target_link_libraries(Pico2W_I2C_Controller
    pico_stdlib
    pico_cyw43_arch_none
    hardware_spi
    hardware_i2c
    hardware_pio
    hardware_dma
)

pico_enable_stdio_uart(Pico2W_I2C_Controller 0)
pico_enable_stdio_usb(Pico2W_I2C_Controller 1)

pico_add_extra_outputs(Pico2W_I2C_Controller)