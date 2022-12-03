#ifndef ACCELEROMETER_GYRO_REGISTERS_HPP_
#define ACCELEROMETER_GYRO_REGISTERS_HPP_

#include "hardware/i2c.h"
// Either "#define i2c i2c0" or "#define i2c i2c1"
#define i2c i2c0

#include <cstdint>

// These are registers for this sensor: https://static6.arrow.com/aropdfconversion/c94e42da32c8f0f5977df326fff91747b7583565/en.dm00557899.pdf

constexpr std::uint8_t ACCEL_GYRO_DEVICE_ADDRESS = 0x6A;
constexpr std::uint8_t ACCEL_GYRO_WHO_AM_I = 0x0F;

#endif