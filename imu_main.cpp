#include <iostream>
#include <iomanip>
#include "imu.hpp"
#include "pico/stdlib.h"

int main() {
    stdio_init_all();

    IMU imu = IMU();
    std::int16_t outputs[3] = {0};

    imu.init();

    while (true) {
        imu.read(outputs);
        std::cout << "X: " << std::setw(5) << outputs[0] << "  Y: " << std::setw(5) << outputs[1] << "  Z: " << std::setw(5) << outputs[2] << std::endl;
    }

    return 0;
}
