#include <iostream>
#include <iomanip>
#include "magnetometer.hpp"
#include "pico/stdlib.h"

int main() {
	// Initialize i2c on whichever core is set in "magnetometer_registers.hpp"
	i2c_init(i2c, 100000);

    gpio_pull_up(0);
    gpio_pull_up(1);
	gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    
    stdio_init_all();

    Magnetometer mag = Magnetometer();
    std::int16_t outputs[3] = {0};

    mag.init();

    while (true) {
        mag.read(outputs);
        std::cout << "X: " << std::setw(12) << outputs[0] << "  Y: " << std::setw(12) << outputs[1] << "  Z: " << std::setw(12) << outputs[2] << std::endl;
    }

    return 0;
}
