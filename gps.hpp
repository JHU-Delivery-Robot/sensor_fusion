#ifndef GPS_HPP_
#define GPS_HPP_

#include <cstdint>
#include "pico/stdlib.h"
#include "minmea.h"

class GPS {
private:
    const uint LED_PIN;
    bool read_line(char* buffer, size_t buffer_size, uart_inst_t *uart_port);
public:
	GPS();
	void read (struct minmea_sentence_gll* output);
	void init ();
};

#endif