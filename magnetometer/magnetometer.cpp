#include "magnetometer.hpp"

#include <iostream>
#include <array>
#include <cstdio>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "magnetometer_registers.hpp"

Magnetometer::Magnetometer() { }

void Magnetometer::read (int16_t* outputs) {
	std::array<std::uint8_t, 6> data;

	i2c_write_blocking(i2c, MAG_DEVICE_ADDRESS, &MAG_OUT_X_L, 1, true);
	i2c_read_blocking(i2c, MAG_DEVICE_ADDRESS, data.data(), 6, false);

	outputs[0] = static_cast<std::int16_t>(static_cast<std::uint16_t>(data[1]) << 8 | static_cast<std::uint16_t>(data[0]));
	outputs[1] = static_cast<std::int16_t>(static_cast<std::uint16_t>(data[3]) << 8 | static_cast<std::uint16_t>(data[2]));
	outputs[2] = static_cast<std::int16_t>(static_cast<std::uint16_t>(data[5]) << 8 | static_cast<std::uint16_t>(data[4]));
}

void Magnetometer::read_with_conversions (float* outputs) {
	std::array<std::uint8_t, 6> data;

	i2c_write_blocking(i2c, MAG_DEVICE_ADDRESS, &MAG_OUT_X_L, 1, true);
	i2c_read_blocking(i2c, MAG_DEVICE_ADDRESS, data.data(), 6, false);

	outputs[0] = static_cast<float>(static_cast<std::int16_t>(static_cast<std::uint16_t>(data[1]) << 8 | static_cast<std::uint16_t>(data[0]))) / 68.42;
	outputs[1] = static_cast<float>(static_cast<std::int16_t>(static_cast<std::uint16_t>(data[3]) << 8 | static_cast<std::uint16_t>(data[2]))) / 68.42;
	outputs[2] = static_cast<float>(static_cast<std::int16_t>(static_cast<std::uint16_t>(data[5]) << 8 | static_cast<std::uint16_t>(data[4]))) / 68.42;
}

void Magnetometer::init () {
	std::uint8_t data[2];

	// Write to register to pull out of sleep mode
	data[0] = MAG_CTRL_REG3;
	// Highest two bits are each 0 by requirement
	// LP (Low-Power Mode) is 0 by default
	// Next two bits are each 0 by requirement
	// SIM set to 0 for 4-wire SPI
	// MD[1:0] set to 00 for continuous-conversion mode (this is what pulls the magnetometer out of sleep mode)
	data[1] = 0b00000000;

	i2c_write_blocking(i2c, MAG_DEVICE_ADDRESS, data, 2, true);
	
	data[0] = MAG_CTRL_REG1;
	// TEMP_EN (Enable Temp Sensor) is 0
	// OM[1:0] set to 00 for low power mode (1000 Hz once FAST_ODR set to 1)
	// DO[2:0] set to 000 (not relevant here)
	// FAST_ODR set to 1 (data rate higher than 80 Hz enabled)
	// ST (self-test) set to 0 for disabled
	data[1] = 0b00000010;

	i2c_write_blocking(i2c, MAG_DEVICE_ADDRESS, data, 2, true);
}