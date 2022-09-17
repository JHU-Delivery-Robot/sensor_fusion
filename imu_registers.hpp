#ifndef IMU_REGISTERS_HPP_
#define IMU_REGISTERS_HPP_

// Either "#define i2c i2c0" or "#define i2c i2c1"
#include "hardware/i2c.h"
#include "imu_gyro_registers.hpp"
#define i2c i2c0

#include <cstdint>

// These are registers for this sensor: https://www.st.com/resource/en/datasheet/lsm6dsox.pdf

constexpr std::uint8_t IMU_CTRL1_XL = 0x10;
constexpr std::uint8_t IMU_OUTX_L_A = 0x28;
constexpr std::uint8_t IMU_OUTX_H_A = 0x29;
constexpr std::uint8_t IMU_OUTY_L_A = 0x2A;
constexpr std::uint8_t IMU_OUTY_H_A = 0x2B;
constexpr std::uint8_t IMU_OUTZ_L_A = 0x2C;
constexpr std::uint8_t IMU_OUTZ_H_A = 0x2D;

#endif