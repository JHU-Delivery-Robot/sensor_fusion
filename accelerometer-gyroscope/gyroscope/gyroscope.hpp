#ifndef GYROSCOPE_HPP_
#define GYROSCOPE_HPP_

#include <cstdint>

#include "gyroscope_registers.hpp"
#include "data_types.hpp"

class Gyroscope {
private:
	const fixed_pt_num gyro_conversion_factor;
public:
	Gyroscope();
	void read (fixed_pt_num* outputs);
	void init ();
};

#endif