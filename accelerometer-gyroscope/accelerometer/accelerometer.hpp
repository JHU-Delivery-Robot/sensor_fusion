#ifndef ACCELEROMETER_HPP_
#define ACCELEROMETER_HPP_

#include <cstdint>

#include "accelerometer_registers.hpp"

class Accelerometer {
public:
	Accelerometer();
	void read (int16_t* outputs);
	void init ();
};

#endif