#include <iostream>
#include <iomanip>
#include <limits>

#include "Fusion.h"
#include "data_types.hpp"
#include "gyroscope.hpp"
#include "accelerometer.hpp"
#include "magnetometer.hpp"

#include "pico/stdlib.h"

#define SAMPLE_RATE (100) // replace this with actual sample rate

int main() {
	// Initialize i2c on whichever core is set in "magnetometer_registers.hpp"
	i2c_init(i2c, 400000);

    //gpio_pull_up(0);
    //gpio_pull_up(1);
	gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);

    stdio_init_all();

    Gyroscope gyro = Gyroscope();
    gyro.init();
    int16_t gyro_outputs[3] = {0};

    Magnetometer mag = Magnetometer();
    mag.init();
    float mag_outputs[3] = {0};

    Accelerometer accel = Accelerometer();
    accel.init();
    int16_t accel_outputs[3] = {0};

    // Define calibration (replace with actual calibration data)
    const FusionMatrix gyroscopeMisalignment = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    const FusionVector gyroscopeSensitivity = {0.00875, 0.00875, 0.00875};
    const FusionVector gyroscopeOffset = {37, 32, 50};
    const FusionMatrix accelerometerMisalignment = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    const FusionVector accelerometerSensitivity = {0.000061, 0.000061, 0.000061};
    const FusionVector accelerometerOffset = {-285, -200, 497};
    const FusionMatrix softIronMatrix = {0.996, 0.046, -0.007, 0.046, 1.004, -0.008, -0.007, -0.008, 1.002};
    const FusionVector hardIronOffset = {-17.81, -7.68, 3.82};

    // Initialise algorithms
    FusionOffset offset;
    FusionAhrs ahrs;

    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .gain = 5,
            .accelerationRejection = 10.0,
            .magneticRejection = 10.0,
            .rejectionTimeout = 10 * SAMPLE_RATE, // 5 seconds

    };
    FusionAhrsSetSettings(&ahrs, &settings);

    absolute_time_t previousTimestamp = get_absolute_time();
    // This loop should repeat each time new gyroscope data is available
    while (true) {
        // Acquire latest sensor data
        absolute_time_t timestamp = get_absolute_time();
        gyro.read(gyro_outputs);
        FusionVector gyroscope = {static_cast<float>(gyro_outputs[0]), static_cast<float>(gyro_outputs[1]), static_cast<float>(gyro_outputs[2])};
        accel.read(accel_outputs);
        FusionVector accelerometer = {static_cast<float>(accel_outputs[0]), static_cast<float>(accel_outputs[1]), static_cast<float>(accel_outputs[2])};
        mag.read_with_conversions(mag_outputs);
        FusionVector magnetometer = {static_cast<float>(mag_outputs[0]), static_cast<float>(mag_outputs[1]), static_cast<float>(mag_outputs[2])};

        // Apply calibration
        gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
        accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
        magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

        // Update gyroscope offset correction algorithm
        gyroscope = FusionOffsetUpdate(&offset, gyroscope);

        // Calculate delta time (in seconds) to account for gyroscope sample clock error
        
        float deltaTime = static_cast<float>(absolute_time_diff_us(previousTimestamp, timestamp)) / 1000000.0;
        previousTimestamp = timestamp;

        // Update gyroscope AHRS algorithm
        FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);
        // FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);

        // Print algorithm outputs
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        
        std::cout << euler.angle.yaw << "," << euler.angle.pitch << "," << euler.angle.roll << std::endl;
        /* std::cout << gyroscope.axis.x << "," << gyroscope.axis.y << "," << gyroscope.axis.z << " -- ";
        std::cout << accelerometer.axis.x << "," << accelerometer.axis.y << "," << accelerometer.axis.z << " -- ";
        std::cout << magnetometer.axis.x << "," << magnetometer.axis.y << "," << magnetometer.axis.z << " -- ";
        std::cout << deltaTime << "," << static_cast<float>(timestamp) << std::endl;*/
    }

    return 0;
}