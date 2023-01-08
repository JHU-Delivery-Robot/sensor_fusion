#include <iostream>
#include <iomanip>

#include "Fusion.hpp"
#include "data_types.hpp"
#include "gyroscope.hpp"
#include "accelerometer.hpp"
#include "magnetometer.hpp"

#include "pico/stdlib.h"

#define SAMPLE_RATE (100) // replace this with actual sample rate

int main() {
	// Initialize i2c on whichever core is set in "magnetometer_registers.hpp"
	i2c_init(i2c, 100000);

    gpio_pull_up(0);
    gpio_pull_up(1);
	gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);

    stdio_init_all();

    Gyroscope gyro = Gyroscope();
    gyro.init();
    int16_t gyro_outputs[3] = {0};

    Accelerometer accel = Accelerometer();
    accel.init();
    int16_t accel_outputs[3] = {0};

    Magnetometer mag = Magnetometer();
    mag.init();
    std::int16_t mag_outputs[3] = {0};

    // Define calibration (replace with actual calibration data)
    const FusionMatrix gyroscopeMisalignment = {fixed_pt_num{1.0}, fixed_pt_num{0.0}, fixed_pt_num{0.0}, fixed_pt_num{0.0}, fixed_pt_num{1.0}, fixed_pt_num{0.0}, fixed_pt_num{0.0}, fixed_pt_num{0.0}, fixed_pt_num{1.0}};
    const FusionVector gyroscopeSensitivity = {fixed_pt_num{0.00875}, fixed_pt_num{0.00875}, fixed_pt_num{0.00875}};
    const FusionVector gyroscopeOffset = {fixed_pt_num{37}, fixed_pt_num{32}, fixed_pt_num{50}};
    const FusionMatrix accelerometerMisalignment = {fixed_pt_num{1.0}, fixed_pt_num{0.0}, fixed_pt_num{0.0}, fixed_pt_num{0.0}, fixed_pt_num{1.0}, fixed_pt_num{0.0}, fixed_pt_num{0.0}, fixed_pt_num{0.0}, fixed_pt_num{1.0}};
    const FusionVector accelerometerSensitivity = {fixed_pt_num{0.000061}, fixed_pt_num{0.000061}, fixed_pt_num{0.000061}};
    const FusionVector accelerometerOffset = {fixed_pt_num{-285}, fixed_pt_num{-200}, fixed_pt_num{497}};
    const FusionMatrix softIronMatrix = {fixed_pt_num{0.985}, fixed_pt_num{0.034}, fixed_pt_num{0.007}, fixed_pt_num{0.034}, fixed_pt_num{1.007}, fixed_pt_num{0.008}, fixed_pt_num{0.007}, fixed_pt_num{0.008}, fixed_pt_num{1.009}};
    const FusionVector hardIronOffset = {fixed_pt_num{-16.4}, fixed_pt_num{-11.12}, fixed_pt_num{-3.26}};

    // Initialise algorithms
    FusionOffset offset;
    FusionAhrs ahrs;

    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .gain = fixed_pt_num{0.5},
            .accelerationRejection = fixed_pt_num{10.0},
            .magneticRejection = fixed_pt_num{20.0},
            .rejectionTimeout = 5 * SAMPLE_RATE, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    absolute_time_t previousTimestamp = get_absolute_time();
    // This loop should repeat each time new gyroscope data is available
    while (true) {
        // Acquire latest sensor data
        absolute_time_t timestamp = get_absolute_time();
        gyro.read(gyro_outputs);
        FusionVector gyroscope = {fixed_pt_num{gyro_outputs[0]}, fixed_pt_num{gyro_outputs[1]}, fixed_pt_num{gyro_outputs[2]}};
        accel.read(accel_outputs);
        FusionVector accelerometer = {fixed_pt_num{accel_outputs[0]}, fixed_pt_num{accel_outputs[1]}, fixed_pt_num{accel_outputs[2]}};
        mag.read(mag_outputs);
        FusionVector magnetometer = {fixed_pt_num{mag_outputs[0]}, fixed_pt_num{mag_outputs[1]}, fixed_pt_num{mag_outputs[2]}};

        // Apply calibration
        gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
        accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
        magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

        // Update gyroscope offset correction algorithm
        gyroscope = FusionOffsetUpdate(&offset, gyroscope);

        // Calculate delta time (in seconds) to account for gyroscope sample clock error
        
        const fixed_pt_num deltaTime = fixed_pt_num{absolute_time_diff_us(previousTimestamp, timestamp)} / fixed_pt_num{1000000.0};
        previousTimestamp = timestamp;

        // Update gyroscope AHRS algorithm
        FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

        // Print algorithm outputs
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        
        std::cout << euler.angle.yaw << "," << euler.angle.pitch << "," << euler.angle.roll << std::endl;
    }

    return 0;
}