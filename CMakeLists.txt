cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk from submodule
# MUST come before project()
include(pico-sdk/pico_sdk_init.cmake)

project(sensor_reading
    VERSION 0.1.0
    LANGUAGES C CXX)

# no non-standard language extensions
set(CMAKE_CXX_EXTENSIONS OFF)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(test
    main.cpp
    minmea.c
)

# pull in common dependencies
target_link_libraries(test pico_stdlib)

# direct serial output to USB rather than UART
pico_enable_stdio_usb(test 1)
pico_enable_stdio_uart(test 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(test)
