#ifndef MAGNETOMETER_REGISTERS_H_
#define MAGNETOMETER_REGISTERS_H_

#include <cstdint>

// These are registers for this sensor: https://www.st.com/resource/en/datasheet/lis3mdl.pdf#page=23

constexpr std::uint8_t MAG_DEVICE_ADDRESS = 0b0011100;
#define MAG_WHO_AM_I 0x0F
#define MAG_CTRL_REG1 0x20
#define MAG_CTRL_REG2 0x21
#define MAG_CTRL_REG3 0x22
#define MAG_CTRL_REG4 0x23
#define MAG_CTRL_REG5 0x24
#define MAG_STATUS_REG 0x27
constexpr std::uint8_t MAG_OUT_X_L = 0x28;
constexpr std::uint8_t MAG_OUT_X_H = 0x29;
constexpr std::uint8_t MAG_OUT_Y_L = 0x2A;
constexpr std::uint8_t MAG_OUT_Y_H = 0x2B;
constexpr std::uint8_t MAG_OUT_Z_L = 0x2C;
constexpr std::uint8_t MAG_OUT_Z_H = 0x2D;
#define MAG_TEMP_OUT_L 0x2E
#define MAG_TEMP_OUT_H 0x2F
#define MAG_INT_CFG 0x30
#define MAG_INT_SRC 0x31
#define MAG_INT_THS_L 0x32
#define MAG_INT_THS_H 0x33

#endif