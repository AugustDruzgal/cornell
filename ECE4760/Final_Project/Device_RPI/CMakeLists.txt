# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# cmake version
cmake_minimum_required(VERSION 3.13)

# include the sdk.cmake file
include(pico_sdk_import.cmake)

# give the project a name (anything you want)
project(Project_Device C CXX ASM)

# initialize the sdk
pico_sdk_init()

add_executable(Project_Device)

target_sources( Project_Device
                PRIVATE
                main.c
                VGA/vga16_graphics_v2.c
                MPU6050/mpu6050.c)

# must match with pio filename and executable name from above
pico_generate_pio_header(Project_Device ${CMAKE_CURRENT_LIST_DIR}/VGA/hsync.pio)
pico_generate_pio_header(Project_Device ${CMAKE_CURRENT_LIST_DIR}/VGA/vsync.pio)
pico_generate_pio_header(Project_Device ${CMAKE_CURRENT_LIST_DIR}/VGA/rgb.pio)

target_link_libraries(Project_Device
    pico_stdlib
    pico_sync
    pico_btstack_ble
    pico_btstack_cyw43
    pico_cyw43_arch_none
    pico_bootsel_via_double_reset
    hardware_irq
    hardware_spi
    hardware_timer
    hardware_sync
    hardware_pio
    hardware_dma
    hardware_pwm
    hardware_adc
    hardware_i2c
    hardware_clocks
    pico_multicore
    )

target_include_directories(Project_Device PRIVATE
    ${CMAKE_CURRENT_LIST_DIR} # For btstack config
    )

pico_btstack_make_gatt_header(Project_Device PRIVATE "${CMAKE_CURRENT_LIST_DIR}/GATT_Service/server_demo_gattfile.gatt")

# create map/bin/hex file etc.
pico_add_extra_outputs(Project_Device)