#include <iostream>

#include "gps.hpp"
#include "pico/stdlib.h"

int main() {
    stdio_init_all();

    GPS gps = GPS();
    struct minmea_sentence_gll gps_output;

    gps.init();

    while (true) {
        gps.read(&gps_output);
        std::cout << "$GLL: (" << gps_output.latitude.value << "/" << gps_output.latitude.scale << ", "
                            << gps_output.longitude.value << "/" << gps_output.longitude.scale << ") "
                            << gps_output.time.hours << ":" << gps_output.time.minutes << ":" << gps_output.time.seconds
                            << gps_output.status << std::endl;
    }

    return 0;
}