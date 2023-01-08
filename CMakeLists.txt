cmake_minimum_required(VERSION 3.15)

# initialize pico-sdk from submodule
# MUST come before project()
include(pico-sdk/pico_sdk_init.cmake)
include_directories(data-types minmea magnetometer accelerometer-gyroscope accelerometer-gyroscope/accelerometer accelerometer-gyroscope/gyroscope gps Fusion)

project(sensor_reading
    VERSION 0.1.0
    LANGUAGES C CXX)

# no non-standard language extensions
set(CMAKE_CXX_EXTENSIONS OFF)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory("Fusion")

# GPS test
add_executable(gps_test
    gps_test.cpp
    gps/gps.cpp
    minmea/minmea.c
)

# pull in common dependencies
target_link_libraries(gps_test pico_stdlib hardware_uart)

# direct serial output to USB rather than UART
pico_enable_stdio_usb(gps_test 1)
pico_enable_stdio_uart(gps_test 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(gps_test)

# Magnetometer
add_executable(magnetometer_test
    magnetometer_test.cpp
    magnetometer/magnetometer.cpp
)

# pull in common dependencies
target_link_libraries(magnetometer_test pico_stdlib hardware_i2c)

# direct serial output to USB rather than UART
pico_enable_stdio_usb(magnetometer_test 1)
pico_enable_stdio_uart(magnetometer_test 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(magnetometer_test)

# Accelerometer
add_executable(accelerometer_offsets
    accelerometer_offsets.cpp
    accelerometer-gyroscope/accelerometer/accelerometer.cpp
)

# pull in common dependencies
target_link_libraries(accelerometer_offsets pico_stdlib hardware_i2c)

# direct serial output to USB rather than UART
pico_enable_stdio_usb(accelerometer_offsets 1)
pico_enable_stdio_uart(accelerometer_offsets 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(accelerometer_offsets)

# Gyroscope
add_executable(gyroscope_offsets
    gyroscope_offsets.cpp
    accelerometer-gyroscope/gyroscope/gyroscope.cpp
)

# pull in common dependencies
target_link_libraries(gyroscope_offsets pico_stdlib hardware_i2c)

# direct serial output to USB rather than UART
pico_enable_stdio_usb(gyroscope_offsets 1)
pico_enable_stdio_uart(gyroscope_offsets 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(gyroscope_offsets)

# Magnetometer
add_executable(ahrs_test
    ahrs_test.cpp
    accelerometer-gyroscope/accelerometer/accelerometer.cpp
    accelerometer-gyroscope/gyroscope/gyroscope.cpp
    magnetometer/magnetometer.cpp
)

# pull in common dependencies
target_link_libraries(ahrs_test pico_stdlib hardware_i2c Fusion)

# direct serial output to USB rather than UART
pico_enable_stdio_usb(ahrs_test 1)
pico_enable_stdio_uart(ahrs_test 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(ahrs_test)