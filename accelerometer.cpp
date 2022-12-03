#include "accelerometer.hpp"

#include <array>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "accelerometer_registers.hpp"

// Value for accel_conversion_factor found from
// https://static6.arrow.com/aropdfconversion/c94e42da32c8f0f5977df326fff91747b7583565/en.dm00557899.pdf
// on page 10. Divided by 10000
Accelerometer::Accelerometer() : accel_conversion_factor(0.000061) {}

void Accelerometer::read (fixed_pt_num* outputs) {
	std::array<std::uint8_t, 6> data;
	std::uint8_t identity[1] = {0};

	i2c_write_blocking(i2c, ACCEL_GYRO_DEVICE_ADDRESS, &ACCEL_OUTX_L_A, 1, true);
	i2c_read_blocking(i2c, ACCEL_GYRO_DEVICE_ADDRESS, data.data(), 6, false);

	outputs[0] = static_cast<std::int16_t>(static_cast<std::uint16_t>(data[1]) << 8 | static_cast<std::uint16_t>(data[0]));
	outputs[0] *= this->accel_conversion_factor;
	outputs[1] = static_cast<std::int16_t>(static_cast<std::uint16_t>(data[3]) << 8 | static_cast<std::uint16_t>(data[2]));
	outputs[1] *= this->accel_conversion_factor;
	outputs[2] = static_cast<std::int16_t>(static_cast<std::uint16_t>(data[5]) << 8 | static_cast<std::uint16_t>(data[4]));
	outputs[2] *= this->accel_conversion_factor;
}

void Accelerometer::init () {
	// Initialize i2c on whichever core is set in "accelerometer_registers.hpp"
	i2c_init(i2c, 100000);

    gpio_pull_up(0);
    gpio_pull_up(1);
	gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);

	// Write to register to pull out of sleep mode
	std::uint8_t data[2];
	data[0] = ACCEL_CTRL1_XL;
	// ODR_XL[3:0] = 0110 => 416 Hz
	// FS[1:0]_XL = 00 => +/- 2g
	// LPF2_XL_EN = 0 (default)
	// last bit = 0 (required)
	data[1] = 0b01100000;

	i2c_write_blocking(i2c, ACCEL_GYRO_DEVICE_ADDRESS, data, 2, true);
}