#include "accelerometer.hpp"

#include <array>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "accelerometer_registers.hpp"

// Value for accel_conversion_factor found from
// https://static6.arrow.com/aropdfconversion/c94e42da32c8f0f5977df326fff91747b7583565/en.dm00557899.pdf
// on page 10. Divided by 10000
Accelerometer::Accelerometer() {}

void Accelerometer::read (int16_t* outputs) {
	std::array<std::uint8_t, 6> data;

	i2c_write_blocking(i2c, ACCEL_GYRO_DEVICE_ADDRESS, &ACCEL_OUTX_L_A, 1, true);
	i2c_read_blocking(i2c, ACCEL_GYRO_DEVICE_ADDRESS, data.data(), 6, false);

	outputs[0] = static_cast<int16_t>(static_cast<std::uint16_t>(data[1]) << 8 | static_cast<std::uint16_t>(data[0]));
	outputs[1] = static_cast<int16_t>(static_cast<std::uint16_t>(data[3]) << 8 | static_cast<std::uint16_t>(data[2]));
	outputs[2] = static_cast<int16_t>(static_cast<std::uint16_t>(data[5]) << 8 | static_cast<std::uint16_t>(data[4]));
}

void Accelerometer::init () {
	// Write to register to pull out of sleep mode
	std::uint8_t data[2];
	data[0] = ACCEL_CTRL1_XL;
	// ODR_XL[3:0] = 1000 => 1.66 kHz
	// FS[1:0]_XL = 00 => +/- 2g
	// LPF2_XL_EN = 0 (default)
	// last bit = 0 (required)
	data[1] = 0b10000000;

	i2c_write_blocking(i2c, ACCEL_GYRO_DEVICE_ADDRESS, data, 2, true);
}