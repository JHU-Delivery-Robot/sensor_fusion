/**
 * @file FusionAhrs.h
 * @author Seb Madgwick
 * @brief AHRS algorithm to combine gyroscope, accelerometer, and magnetometer
 * measurements into a single measurement of orientation relative to the Earth.
 */

#ifndef FUSION_AHRS_HPP
#define FUSION_AHRS_HPP

//------------------------------------------------------------------------------
// Includes

#include "FusionMath.hpp"
#include <cstdbool>

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief AHRS algorithm settings.
 */
typedef struct {
    fixed_pt_num gain;
    fixed_pt_num accelerationRejection;
    fixed_pt_num magneticRejection;
    unsigned int rejectionTimeout;
} FusionAhrsSettings;

/**
 * @brief AHRS algorithm structure.  Structure members are used internally and
 * must not be accessed by the application.
 */
typedef struct {
    FusionAhrsSettings settings;
    FusionQuaternion quaternion;
    FusionVector accelerometer;
    bool initialising;
    fixed_pt_num rampedGain;
    fixed_pt_num rampedGainStep;
    FusionVector halfAccelerometerFeedback;
    FusionVector halfMagnetometerFeedback;
    bool accelerometerIgnored;
    unsigned int accelerationRejectionTimer;
    bool accelerationRejectionTimeout;
    bool magnetometerIgnored;
    unsigned int magneticRejectionTimer;
    bool magneticRejectionTimeout;
} FusionAhrs;

/**
 * @brief AHRS algorithm internal states.
 */
typedef struct {
    fixed_pt_num accelerationError;
    bool accelerometerIgnored;
    fixed_pt_num accelerationRejectionTimer;
    fixed_pt_num magneticError;
    bool magnetometerIgnored;
    fixed_pt_num magneticRejectionTimer;
} FusionAhrsInternalStates;

/**
 * @brief AHRS algorithm flags.
 */
typedef struct {
    bool initialising;
    bool accelerationRejectionWarning;
    bool accelerationRejectionTimeout;
    bool magneticRejectionWarning;
    bool magneticRejectionTimeout;
} FusionAhrsFlags;

//------------------------------------------------------------------------------
// Function declarations

void FusionAhrsInitialise(FusionAhrs *const ahrs);

void FusionAhrsReset(FusionAhrs *const ahrs);

void FusionAhrsSetSettings(FusionAhrs *const ahrs, const FusionAhrsSettings *const settings);

void FusionAhrsUpdate(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const FusionVector magnetometer, const fixed_pt_num deltaTime);

void FusionAhrsUpdateNoMagnetometer(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const fixed_pt_num deltaTime);

void FusionAhrsUpdateExternalHeading(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const fixed_pt_num heading, const fixed_pt_num deltaTime);

FusionQuaternion FusionAhrsGetQuaternion(const FusionAhrs *const ahrs);

FusionVector FusionAhrsGetLinearAcceleration(const FusionAhrs *const ahrs);

FusionVector FusionAhrsGetEarthAcceleration(const FusionAhrs *const ahrs);

FusionAhrsInternalStates FusionAhrsGetInternalStates(const FusionAhrs *const ahrs);

FusionAhrsFlags FusionAhrsGetFlags(FusionAhrs *const ahrs);

void FusionAhrsSetHeading(FusionAhrs *const ahrs, const fixed_pt_num heading);

#endif

//------------------------------------------------------------------------------
// End of file
