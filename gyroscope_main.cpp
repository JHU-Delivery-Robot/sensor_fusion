#include <iostream>
#include <iomanip>

#include "gyroscope.hpp"
#include "pico/stdlib.h"

int main() {
    stdio_init_all();

    Gyroscope gyro = Gyroscope();
    fixed_pt_num outputs[3] = {0};

    gyro.init();

    while (true) {
        gyro.read(outputs);
        std::cout << "X: " << std::setw(5) << outputs[0] << "  Y: " << std::setw(5) << outputs[1] << "  Z: " << std::setw(5) << outputs[2] << std::endl;
    }

    return 0;
}
