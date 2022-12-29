#ifndef GYROSCOPE_HPP_
#define GYROSCOPE_HPP_

#include <cstdint>

#include "gyroscope_registers.hpp"

class Gyroscope {
public:
	Gyroscope();
	void read (int16_t* outputs);
	void init ();
};

#endif