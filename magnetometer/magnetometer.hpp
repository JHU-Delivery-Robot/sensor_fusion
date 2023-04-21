#ifndef MAGNETOMETER_HPP_
#define MAGNETOMETER_HPP_

#include "magnetometer_registers.hpp"
#include <cstdint>

class Magnetometer {
public:
	Magnetometer();
	void read (int16_t* outputs);
	void read_with_conversions (float* outputs);
	void init ();
};

#endif