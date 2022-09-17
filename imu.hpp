#ifndef IMU_HPP_
#define IMU_HPP_

#include "magnetometer_registers.hpp"
#include <cstdint>

class IMU {
public:
	IMU();
	void read (int16_t* outputs);
	void init ();
};

#endif