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

	std::int16_t x_current_reading = static_cast<std::int16_t>(static_cast<std::uint16_t>(data[1]) << 8 | static_cast<std::uint16_t>(data[0]));
	std::int16_t y_current_reading = static_cast<std::int16_t>(static_cast<std::uint16_t>(data[3]) << 8 | static_cast<std::uint16_t>(data[2]));
	std::int16_t z_current_reading = static_cast<std::int16_t>(static_cast<std::uint16_t>(data[5]) << 8 | static_cast<std::uint16_t>(data[4]));

	outputs[0] = x_current_reading;
	outputs[1] = y_current_reading;
	outputs[2] = z_current_reading;
}

void Magnetometer::init () {
	// Initialize i2c on whichever core is set in "magnetometer_registers.hpp"
	i2c_init(i2c, 100000);

    gpio_pull_up(0);
    gpio_pull_up(1);
	gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);

	// Write to register to pull out of sleep mode
	std::uint8_t data[2];
	data[0] = MAG_CTRL_REG3;
	data[1] = 0x00;

	i2c_write_blocking(i2c, MAG_DEVICE_ADDRESS, data, 2, true);
}