#include "gyroscope.hpp"

#include <array>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "gyroscope_registers.hpp"

// Value for below found from
// https://static6.arrow.com/aropdfconversion/c94e42da32c8f0f5977df326fff91747b7583565/en.dm00557899.pdf
// on page 10
Gyroscope::Gyroscope() { }

void Gyroscope::read (int16_t* outputs) {
	std::array<std::uint8_t, 6> data;

	i2c_write_blocking(i2c, ACCEL_GYRO_DEVICE_ADDRESS, &GYRO_OUTX_L_G, 1, true);
	i2c_read_blocking(i2c, ACCEL_GYRO_DEVICE_ADDRESS, data.data(), 6, false);

	outputs[0] = static_cast<std::int16_t>(static_cast<std::uint16_t>(data[1]) << 8 | static_cast<std::uint16_t>(data[0]));
	outputs[1] = static_cast<std::int16_t>(static_cast<std::uint16_t>(data[3]) << 8 | static_cast<std::uint16_t>(data[2]));
	outputs[2] = static_cast<std::int16_t>(static_cast<std::uint16_t>(data[5]) << 8 | static_cast<std::uint16_t>(data[4]));
}

void Gyroscope::init () {
	// Initialize i2c on whichever core is set in "magnetometer_registers.hpp"
	i2c_init(i2c, 100000);

    gpio_pull_up(0);
    gpio_pull_up(1);
	gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);

	// Write to register to pull out of sleep mode
	std::uint8_t data[2];
	data[0] = GYRO_CTRL2_G;
	// ODR_G[3:0] = 1000 => 1.66 kHz
	// FS[1:0]_G = 00 => +/- 250 dps
	// FS_125 = 0 (select angular rate sensitivity through FS[1:0]_G)
	// last bit = 0 (required)
	data[1] = 0b10000000;

	i2c_write_blocking(i2c, ACCEL_GYRO_DEVICE_ADDRESS, data, 2, true);
}