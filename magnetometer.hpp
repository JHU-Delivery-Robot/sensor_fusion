#include "magnetometer_registers.hpp"
#include <cstdint>

class Magnetometer {
private:

	std::uint16_t current_index;
	std::uint16_t max_latest_readings;
	std::int16_t x_latest_readings[10];
	std::int16_t y_latest_readings[10];
	std::int16_t z_latest_readings[10];
	std::int16_t x_total_latest_readings;
	std::int16_t y_total_latest_readings;
	std::int16_t z_total_latest_readings;
	std::uint8_t timeout;

public:

	Magnetometer(std::uint8_t timeout);

	void read (int16_t* outputs);

	void reset ();

	void print_test ();
};