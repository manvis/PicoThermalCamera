# DO NOT FORGET TO SET -DPICO_PLATFORM=rp2350 when setting up the CMake project!

cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
include(pimoroni_pico_import.cmake)
project(thermalCam C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# WAY too many warnings from the Pico SDK
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -fdiagnostics-color=always")

pico_sdk_init()

include(${PIMORONI_PICO_PATH}/libraries/pico_display/pico_display.cmake)
include(${PIMORONI_PICO_PATH}/libraries/pico_graphics/pico_graphics.cmake)
include(${PIMORONI_PICO_PATH}/drivers/st7789/st7789.cmake)
include(${PIMORONI_PICO_PATH}/drivers/button/button.cmake)
include(${PIMORONI_PICO_PATH}/drivers/rgbled/rgbled.cmake)
include(${PIMORONI_PICO_PATH}/common/pimoroni_bus.cmake)

add_executable(thermalCam
    main.cpp
    MLX90640_I2C_PicoDriver.cpp
    dependencies/mlx90640-library/functions/MLX90640_API.cpp
)

include_directories(dependencies/mlx90640-library/headers)

pico_enable_stdio_usb(thermalCam 1)
pico_enable_stdio_uart(thermalCam 1)
pico_add_extra_outputs(thermalCam)
target_link_libraries(thermalCam
        pico_stdlib
        pico_display
        pico_graphics
        hardware_adc
        hardware_spi
        hardware_pwm
        hardware_dma
        hardware_pio
        hardware_i2c
        rgbled
        button
        st7789)
