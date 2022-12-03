#include <iostream>
#include <iomanip>

#include "accelerometer.hpp"
#include "pico/stdlib.h"

int main() {
    stdio_init_all();
    
    int res = 0;

    Accelerometer accel = Accelerometer();
    fixed_pt_num outputs[3] = {0};

    accel.init();

    uint64_t current_time = 0;

    while (true) {
        accel.read(outputs);
        std::cout << "X: " << std::setw(5) << outputs[0] << "  Y: " << std::setw(5) << outputs[1] << "  Z: " << std::setw(5) << outputs[2] << std::endl;
    }

    res = 0;

    return 0;
}
