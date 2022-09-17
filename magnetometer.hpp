#include "magnetometer_registers.hpp"
#include <cstdint>

class Magnetometer {
public:
	Magnetometer();
	void read (int16_t* outputs);
	void init ();
};