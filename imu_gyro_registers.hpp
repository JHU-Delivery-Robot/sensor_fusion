#ifndef IMU_GYRO_REGISTERS_HPP_
#define IMU_GYRO_REGISTERS_HPP_

// Either "#define i2c i2c0" or "#define i2c i2c1"
#include "hardware/i2c.h"
#define i2c i2c0

#include <cstdint>

// These are registers for this sensor: https://www.st.com/resource/en/datasheet/lsm6dsox.pdf

constexpr std::uint8_t IMU_GYRO_DEVICE_ADDRESS = 0x6A;
constexpr std::uint8_t IMU_GYRO_WHO_AM_I = 0x0F;

#endif