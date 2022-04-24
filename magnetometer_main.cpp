#include <iostream>
#include "magnetometer.hpp"
#include "pico/stdlib.h"

int main() {
    stdio_init_all();

    Magnetometer mag = Magnetometer(10);
    std::int16_t outputs[3] = {0};

    mag.init();

    while (true) {
        mag.read(outputs);
        std::cout << "X: " << outputs[0] << std::endl << "Y: " << outputs[1] << std::endl << "Z: " << outputs[2] << std::endl;
    }

    return 0;
}
