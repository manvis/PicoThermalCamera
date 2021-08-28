cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
include(pimoroni_pico_import.cmake)
project(thermalCam C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

include(${PIMORONI_PICO_PATH}/libraries/pico_display/pico_display.cmake)

add_executable(thermalCam
    main.cpp
)

pico_enable_stdio_usb(thermalCam 1)
pico_enable_stdio_uart(thermalCam 1)
pico_add_extra_outputs(thermalCam)
target_link_libraries(thermalCam pico_stdlib hardware_spi hardware_pwm hardware_adc hardware_dma pico_display)
