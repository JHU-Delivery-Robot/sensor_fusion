#ifndef MAGNETOMETER_REGISTERS_HPP_
#define MAGNETOMETER_REGISTERS_HPP_

// Either "#define i2c i2c0" or "#define i2c i2c1"
#include "hardware/i2c.h"
#define i2c i2c0

#include <cstdint>

// These are registers for this sensor: https://www.st.com/resource/en/datasheet/lis3mdl.pdf#page=23

constexpr std::uint8_t MAG_DEVICE_ADDRESS = 0x1C;
constexpr std::uint8_t MAG_WHO_AM_I = 0x0F;
constexpr std::uint8_t MAG_CTRL_REG1 = 0x20;
constexpr std::uint8_t MAG_CTRL_REG2 = 0x21;
constexpr std::uint8_t MAG_CTRL_REG3 = 0x22;
constexpr std::uint8_t MAG_CTRL_REG4 = 0x23;
constexpr std::uint8_t MAG_CTRL_REG5 = 0x24;
constexpr std::uint8_t MAG_STATUS_REG = 0x27;
constexpr std::uint8_t MAG_OUT_X_L = 0x28;
constexpr std::uint8_t MAG_OUT_X_H = 0x29;
constexpr std::uint8_t MAG_OUT_Y_L = 0x2A;
constexpr std::uint8_t MAG_OUT_Y_H = 0x2B;
constexpr std::uint8_t MAG_OUT_Z_L = 0x2C;
constexpr std::uint8_t MAG_OUT_Z_H = 0x2D;
constexpr std::uint8_t MAG_TEMP_OUT_L = 0x2E;
constexpr std::uint8_t MAG_TEMP_OUT_H = 0x2F;
constexpr std::uint8_t MAG_INT_CFG = 0x30;
constexpr std::uint8_t MAG_INT_SRC = 0x31;
constexpr std::uint8_t MAG_INT_THS_L = 0x32;
constexpr std::uint8_t MAG_INT_THS_H = 0x33;

#endif