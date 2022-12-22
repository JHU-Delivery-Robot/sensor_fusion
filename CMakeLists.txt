cmake_minimum_required(VERSION 3.15)

# initialize pico-sdk from submodule
# MUST come before project()
include(pico-sdk/pico_sdk_init.cmake)
include_directories(data-types minmea magnetometer accelerometer-gyroscope accelerometer-gyroscope/accelerometer accelerometer-gyroscope/gyroscope gps)

project(sensor_reading
    VERSION 0.1.0
    LANGUAGES C CXX)

# no non-standard language extensions
set(CMAKE_CXX_EXTENSIONS OFF)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# GPS test
add_executable(gps
    gps_main.cpp
    gps/gps.cpp
    minmea/minmea.c
)

# pull in common dependencies
target_link_libraries(gps pico_stdlib hardware_uart)

# direct serial output to USB rather than UART
pico_enable_stdio_usb(gps 1)
pico_enable_stdio_uart(gps 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(gps)

# Magnetometer
add_executable(magnetometer
    magnetometer_main.cpp
    magnetometer/magnetometer.cpp
)

# pull in common dependencies
target_link_libraries(magnetometer pico_stdlib hardware_i2c)

# direct serial output to USB rather than UART
pico_enable_stdio_usb(magnetometer 1)
pico_enable_stdio_uart(magnetometer 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(magnetometer)

# Accelerometer
add_executable(accelerometer
    accelerometer_main.cpp
    accelerometer-gyroscope/accelerometer/accelerometer.cpp
)

# pull in common dependencies
target_link_libraries(accelerometer pico_stdlib hardware_i2c)

# direct serial output to USB rather than UART
pico_enable_stdio_usb(accelerometer 1)
pico_enable_stdio_uart(accelerometer 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(accelerometer)

# Gyroscope
add_executable(gyroscope
    gyroscope_main.cpp
    accelerometer-gyroscope/gyroscope/gyroscope.cpp
)

# pull in common dependencies
target_link_libraries(gyroscope pico_stdlib hardware_i2c)

# direct serial output to USB rather than UART
pico_enable_stdio_usb(gyroscope 1)
pico_enable_stdio_uart(gyroscope 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(gyroscope)
