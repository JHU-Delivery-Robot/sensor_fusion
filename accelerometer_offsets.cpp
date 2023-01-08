#include <iostream>
#include <iomanip>

#include "accelerometer.hpp"
#include "pico/stdlib.h"

int main() {
	// Initialize i2c on whichever core is set in "magnetometer_registers.hpp"
	i2c_init(i2c, 100000);

    gpio_pull_up(0);
    gpio_pull_up(1);
	gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    
    stdio_init_all();

    Accelerometer accel = Accelerometer();
    int16_t outputs[3] = {0};
    int x = 0;
    int y = 0;
    int z = 0;
    int n = 0;
    int n_data_points = 0;

    accel.init();

    while (true) {
        accel.read(outputs);
        n_data_points++;
        if (n_data_points > 100) {
            n++;
            x += outputs[0];
            y += outputs[1];
            z += outputs[2];

            std::cout << "X: " << std::setw(12) << x / n << "  Y: " << std::setw(12) << y / n << "  Z: " << std::setw(12) << z / n - 16393 << std::endl;
        }
    }

    return 0;
}
