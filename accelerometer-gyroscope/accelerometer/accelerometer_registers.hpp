#ifndef ACCELEROMETER_REGISTERS_HPP_
#define ACCELEROMETER_REGISTERS_HPP_

#include "hardware/i2c.h"
// Either "#define i2c i2c0" or "#define i2c i2c1"
#define i2c i2c0
#include "accelerometer_gyro_registers.hpp"

#include <cstdint>

// These are registers for this sensor: https://static6.arrow.com/aropdfconversion/c94e42da32c8f0f5977df326fff91747b7583565/en.dm00557899.pdf

constexpr std::uint8_t ACCEL_CTRL1_XL = 0x10;
constexpr std::uint8_t ACCEL_OUTX_L_A = 0x28;
constexpr std::uint8_t ACCEL_OUTX_H_A = 0x29;
constexpr std::uint8_t ACCEL_OUTY_L_A = 0x2A;
constexpr std::uint8_t ACCEL_OUTY_H_A = 0x2B;
constexpr std::uint8_t ACCEL_OUTZ_L_A = 0x2C;
constexpr std::uint8_t ACCEL_OUTZ_H_A = 0x2D;

#endif