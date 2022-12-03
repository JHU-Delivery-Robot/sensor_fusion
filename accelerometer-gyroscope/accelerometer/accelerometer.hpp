#ifndef ACCELEROMETER_HPP_
#define ACCELEROMETER_HPP_

#include <cstdint>

#include "accelerometer_registers.hpp"
#include "data_types.hpp"

class Accelerometer {
private:
	const fixed_pt_num accel_conversion_factor;
public:
	Accelerometer();
	void read (fixed_pt_num* outputs);
	void init ();
};

#endif