#ifndef GYROSCOPE_REGISTERS_HPP_
#define GYROSCOPE_REGISTERS_HPP_

#include "accelerometer_gyro_registers.hpp"
// Either "#define i2c i2c0" or "#define i2c i2c1"
#define i2c i2c0
#include "hardware/i2c.h"

#include <cstdint>

// These are registers for this sensor: https://static6.arrow.com/aropdfconversion/c94e42da32c8f0f5977df326fff91747b7583565/en.dm00557899.pdf

constexpr std::uint8_t GYRO_CTRL2_G = 0x11;
constexpr std::uint8_t GYRO_OUTX_L_G = 0x22;
constexpr std::uint8_t GYRO_OUTX_H_G = 0x23;
constexpr std::uint8_t GYRO_OUTY_L_G = 0x24;
constexpr std::uint8_t GYRO_OUTY_H_G = 0x25;
constexpr std::uint8_t GYRO_OUTZ_L_G = 0x26;
constexpr std::uint8_t GYRO_OUTZ_H_G = 0x27;

#endif