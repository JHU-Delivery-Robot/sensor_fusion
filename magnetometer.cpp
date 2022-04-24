#include "magnetometer_registers.hpp"
#include "magnetometer.hpp"
#include <iostream>
#include "hardware/i2c.h"

Magnetometer::Magnetometer(std::uint8_t timeout) : current_index(0),
						max_latest_readings(10),
						x_latest_readings{0},
						y_latest_readings{0},
						z_latest_readings{0},
						x_total_latest_readings(0),
						y_total_latest_readings(0),
						z_total_latest_readings(0),
						timeout(timeout) {
	i2c_init(i2c_default, 9600);
}

void Magnetometer::read (int16_t* outputs) {
	std::uint8_t low = 0;
	std::uint8_t high = 0;

	i2c_write_timeout_us(i2c_default, MAG_DEVICE_ADDRESS, &MAG_OUT_X_L, 1, false, timeout);
	i2c_read_timeout_us(i2c_default, MAG_DEVICE_ADDRESS, &low, 1, false, timeout);
	i2c_write_timeout_us(i2c_default, MAG_DEVICE_ADDRESS, &MAG_OUT_X_H, 1, false, timeout);
	i2c_read_timeout_us(i2c_default, MAG_DEVICE_ADDRESS, &high, 1, false, timeout);

	int16_t x_current_reading = (high << 8) | low;

	i2c_write_timeout_us(i2c_default, MAG_DEVICE_ADDRESS, &MAG_OUT_Y_L, 1, false, timeout);
	i2c_read_timeout_us(i2c_default, MAG_DEVICE_ADDRESS, &low, 1, false, timeout);
	i2c_write_timeout_us(i2c_default, MAG_DEVICE_ADDRESS, &MAG_OUT_Y_H, 1, false, timeout);
	i2c_read_timeout_us(i2c_default, MAG_DEVICE_ADDRESS, &high, 1, false, timeout);

	int16_t y_current_reading = (high << 8) | low;

	i2c_write_timeout_us(i2c_default, MAG_DEVICE_ADDRESS, &MAG_OUT_Z_L, 1, false, timeout);
	i2c_read_timeout_us(i2c_default, MAG_DEVICE_ADDRESS, &low, 1, false, timeout);
	i2c_write_timeout_us(i2c_default, MAG_DEVICE_ADDRESS, &MAG_OUT_Z_H, 1, false, timeout);
	i2c_read_timeout_us(i2c_default, MAG_DEVICE_ADDRESS, &high, 1, false, timeout);

	int16_t z_current_reading = (high << 8) | low;

	x_total_latest_readings -= x_latest_readings[current_index];
	x_total_latest_readings += x_current_reading;
	x_latest_readings[current_index] = x_current_reading;

	y_total_latest_readings -= y_latest_readings[current_index];
	y_total_latest_readings += y_current_reading;
	y_latest_readings[current_index] = y_current_reading;

	z_total_latest_readings -= z_latest_readings[current_index];
	z_total_latest_readings += z_current_reading;
	z_latest_readings[current_index] = z_current_reading;

	if (current_index < max_latest_readings - 1) {
		current_index++;
	} else {
		current_index = 0;
	}

	outputs[0] = x_total_latest_readings / max_latest_readings;
	outputs[1] = y_total_latest_readings / max_latest_readings;
	outputs[2] = z_total_latest_readings / max_latest_readings;
}

void Magnetometer::reset () {
	current_index = 0;
	for (int i = 0; i < 10; i++) {
		x_latest_readings[i] = 0;
		y_latest_readings[i] = 0;
		z_latest_readings[i] = 0;
	}
	x_total_latest_readings = 0;
	y_total_latest_readings = 0;
	z_total_latest_readings = 0;
}

void Magnetometer::print_test () {
	std::cout << "x:" << std::endl;
	for (int i = 0; i < 10; i++)
		std::cout << x_latest_readings[i] << " ";
	std::cout << std::endl << "y:" << std::endl;
	for (int i = 0; i < 10; i++)
		std::cout << y_latest_readings[i] << " ";
	std::cout << std::endl << "z:" << std::endl;
	for (int i = 0; i < 10; i++)
		std::cout << z_latest_readings[i] << " ";
	std::cout << std::endl;
}

/*int main () {
	Magnetometer mag = Magnetometer(10);
	std::int16_t outputs[3] = {0};
	for (int i = 0; i < 9; i++)
		mag.read(outputs);
	mag.print_test();
	std::cout << outputs[2] << std::endl << std::endl;
	mag.read(outputs);
	mag.print_test();
	std::cout << outputs[2] << std::endl << std::endl;
	mag.read(outputs);
	mag.print_test();
	std::cout << outputs[2] << std::endl << std::endl;
	return 0;
}*/