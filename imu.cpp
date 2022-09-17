#include "imu.hpp"

#include <iostream>
#include <array>
#include <cstdio>
#include <iostream>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "imu_registers.hpp"

IMU::IMU() { }

void IMU::read (int16_t* outputs) {
	std::array<std::uint8_t, 6> data;
	std::uint8_t identity[1] = {0};

	i2c_write_blocking(i2c, IMU_GYRO_DEVICE_ADDRESS, &IMU_OUTX_L_A, 1, true);
	i2c_read_blocking(i2c, IMU_GYRO_DEVICE_ADDRESS, data.data(), 6, false);

	outputs[0] = static_cast<std::int16_t>(static_cast<std::uint16_t>(data[1]) << 8 | static_cast<std::uint16_t>(data[0]));
	outputs[1] = static_cast<std::int16_t>(static_cast<std::uint16_t>(data[3]) << 8 | static_cast<std::uint16_t>(data[2]));
	outputs[2] = static_cast<std::int16_t>(static_cast<std::uint16_t>(data[5]) << 8 | static_cast<std::uint16_t>(data[4]));
}

void IMU::init () {
	// Initialize i2c on whichever core is set in "imu_registers.hpp"
	i2c_init(i2c, 100000);

    gpio_pull_up(0);
    gpio_pull_up(1);
	gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);

	// Write to register to pull out of sleep mode
	std::uint8_t data[2];
	data[0] = IMU_CTRL1_XL;
	data[1] = 0x60;

	i2c_write_blocking(i2c, IMU_GYRO_DEVICE_ADDRESS, data, 2, true);
}