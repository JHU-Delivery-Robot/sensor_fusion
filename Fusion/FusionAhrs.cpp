/**
 * @file FusionAhrs.c
 * @author Seb Madgwick
 * @brief AHRS algorithm to combine gyroscope, accelerometer, and magnetometer
 * measurements into a single measurement of orientation relative to the Earth.
 */

//------------------------------------------------------------------------------
// Includes

#include "FusionAhrs.hpp"
#include "FusionCompass.hpp"
using fpm::atan2;
using fpm::cos;
using fpm::pow;
using fpm::sin;

//------------------------------------------------------------------------------
// Definitions

fixed_pt_num FLT_MAX {2};

/**
 * @brief Initial gain used during the initialisation.
 */
fixed_pt_num INITIAL_GAIN {10.0};

/**
 * @brief Initialisation period in seconds.
 */
fixed_pt_num INITIALISATION_PERIOD {3.0};

//------------------------------------------------------------------------------
// Functions

/**
 * @brief Initialises the AHRS algorithm structure.
 * @param ahrs AHRS algorithm structure.
 */
void FusionAhrsInitialise(FusionAhrs *const ahrs) {
    const FusionAhrsSettings settings = {
            .gain = fixed_pt_num{0.5},
            .accelerationRejection = fixed_pt_num{90.0},
            .magneticRejection = fixed_pt_num{90.0},
            .rejectionTimeout = 0,
    };
    FusionAhrsSetSettings(ahrs, &settings);
    FusionAhrsReset(ahrs);
}

/**
 * @brief Resets the AHRS algorithm.  This is equivalent to reinitialising the
 * algorithm while maintaining the current settings.
 * @param ahrs AHRS algorithm structure.
 */
void FusionAhrsReset(FusionAhrs *const ahrs) {
    ahrs->quaternion = FUSION_IDENTITY_QUATERNION;
    ahrs->accelerometer = FUSION_VECTOR_ZERO;
    ahrs->initialising = true;
    ahrs->rampedGain = INITIAL_GAIN;
    ahrs->halfAccelerometerFeedback = FUSION_VECTOR_ZERO;
    ahrs->halfMagnetometerFeedback = FUSION_VECTOR_ZERO;
    ahrs->accelerometerIgnored = false;
    ahrs->accelerationRejectionTimer = 0;
    ahrs->accelerationRejectionTimeout = false;
    ahrs->magnetometerIgnored = false;
    ahrs->magneticRejectionTimer = 0;
    ahrs->magneticRejectionTimeout = false;
}

/**
 * @brief Sets the AHRS algorithm settings.
 * @param ahrs AHRS algorithm structure.
 * @param settings Settings.
 */
void FusionAhrsSetSettings(FusionAhrs *const ahrs, const FusionAhrsSettings *const settings) {
    ahrs->settings.gain = settings->gain;
    if ((settings->accelerationRejection == fixed_pt_num{0.0}) || (settings->rejectionTimeout == 0)) {
        ahrs->settings.accelerationRejection = FLT_MAX;
    } else {
        ahrs->settings.accelerationRejection = pow(fixed_pt_num{0.5} * sin(FusionDegreesToRadians(settings->accelerationRejection)), 2);
    }
    if ((settings->magneticRejection == fixed_pt_num{0.0}) || (settings->rejectionTimeout == 0)) {
        ahrs->settings.magneticRejection = FLT_MAX;
    } else {
        ahrs->settings.magneticRejection = pow(fixed_pt_num{0.5} * sin(FusionDegreesToRadians(settings->magneticRejection)), 2);
    }
    ahrs->settings.rejectionTimeout = settings->rejectionTimeout;
    if (ahrs->initialising == false) {
        ahrs->rampedGain = ahrs->settings.gain;
    }
    ahrs->rampedGainStep = (INITIAL_GAIN - ahrs->settings.gain) / INITIALISATION_PERIOD;
}

/**
 * @brief Updates the AHRS algorithm using the gyroscope, accelerometer, and
 * magnetometer measurements.
 * @param ahrs AHRS algorithm structure.
 * @param gyroscope Gyroscope measurement in degrees per second.
 * @param accelerometer Accelerometer measurement in g.
 * @param magnetometer Magnetometer measurement in arbitrary units.
 * @param deltaTime Delta time in seconds.
 */
void FusionAhrsUpdate(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const FusionVector magnetometer, const fixed_pt_num deltaTime) {
#define Q ahrs->quaternion.element

    // Store accelerometer
    ahrs->accelerometer = accelerometer;

    // Ramp down gain during initialisation
    if (ahrs->initialising == true) {
        ahrs->rampedGain -= ahrs->rampedGainStep * deltaTime;
        if (ahrs->rampedGain < ahrs->settings.gain) {
            ahrs->rampedGain = ahrs->settings.gain;
            ahrs->initialising = false;
            ahrs->accelerationRejectionTimeout = false;
        }
    }

    // Calculate direction of gravity indicated by algorithm
    FusionVector halfGravity; // should be const
    halfGravity.axis.x = Q.x * Q.z - Q.w * Q.y;
    halfGravity.axis.y = Q.y * Q.z + Q.w * Q.x;
    halfGravity.axis.z = Q.w * Q.w - fixed_pt_num{0.5} + Q.z * Q.z;
    // third column of transposed rotation matrix scaled by 0.5

    // Calculate accelerometer feedback
    FusionVector halfAccelerometerFeedback = FUSION_VECTOR_ZERO;
    ahrs->accelerometerIgnored = true;
    if (FusionVectorIsZero(accelerometer) == false) {

        // Enter acceleration recovery state if acceleration rejection times out
        if (ahrs->accelerationRejectionTimer > ahrs->settings.rejectionTimeout) {
            const FusionQuaternion quaternion = ahrs->quaternion;
            FusionAhrsReset(ahrs);
            ahrs->quaternion = quaternion;
            ahrs->accelerationRejectionTimer = 0;
            ahrs->accelerationRejectionTimeout = true;
        }

        // Calculate accelerometer feedback scaled by 0.5
        ahrs->halfAccelerometerFeedback = FusionVectorCrossProduct(FusionVectorNormalise(accelerometer), halfGravity);

        // Ignore accelerometer if acceleration distortion detected
        if ((ahrs->initialising == true) || (FusionVectorMagnitudeSquared(ahrs->halfAccelerometerFeedback) <= ahrs->settings.accelerationRejection)) {
            halfAccelerometerFeedback = ahrs->halfAccelerometerFeedback;
            ahrs->accelerometerIgnored = false;
            ahrs->accelerationRejectionTimer -= ahrs->accelerationRejectionTimer >= 10 ? 10 : 0;
        } else {
            ahrs->accelerationRejectionTimer++;
        }
    }

    // Calculate magnetometer feedback
    FusionVector halfMagnetometerFeedback = FUSION_VECTOR_ZERO;
    ahrs->magnetometerIgnored = true;
    if (FusionVectorIsZero(magnetometer) == false) {

        // Set to compass heading if magnetic rejection times out
        ahrs->magneticRejectionTimeout = false;
        if (ahrs->magneticRejectionTimer > ahrs->settings.rejectionTimeout) {
            FusionAhrsSetHeading(ahrs, FusionCompassCalculateHeading(halfGravity, magnetometer));
            ahrs->magneticRejectionTimer = 0;
            ahrs->magneticRejectionTimeout = true;
        }

        // Compute direction of west indicated by algorithm
        FusionVector halfWest; // should be const
        halfWest.axis.x = Q.x * Q.y + Q.w * Q.z;
        halfWest.axis.y = Q.w * Q.w - fixed_pt_num{0.5} + Q.y * Q.y;
        halfWest.axis.z = Q.y * Q.z - Q.w * Q.x;
        // second column of transposed rotation matrix scaled by 0.5

        // Calculate magnetometer feedback scaled by 0.5
        ahrs->halfMagnetometerFeedback = FusionVectorCrossProduct(FusionVectorNormalise(FusionVectorCrossProduct(halfGravity, magnetometer)), halfWest);

        // Ignore magnetometer if magnetic distortion detected
        if ((ahrs->initialising == true) || (FusionVectorMagnitudeSquared(ahrs->halfMagnetometerFeedback) <= ahrs->settings.magneticRejection)) {
            halfMagnetometerFeedback = ahrs->halfMagnetometerFeedback;
            ahrs->magnetometerIgnored = false;
            ahrs->magneticRejectionTimer -= ahrs->magneticRejectionTimer >= 10 ? 10 : 0;
        } else {
            ahrs->magneticRejectionTimer++;
        }
    }

    // Convert gyroscope to radians per second scaled by 0.5
    const FusionVector halfGyroscope = FusionVectorMultiplyScalar(gyroscope, FusionDegreesToRadians(fixed_pt_num{0.5}));

    // Apply feedback to gyroscope
    const FusionVector adjustedHalfGyroscope = FusionVectorAdd(halfGyroscope, FusionVectorMultiplyScalar(FusionVectorAdd(halfAccelerometerFeedback, halfMagnetometerFeedback), ahrs->rampedGain));

    // Integrate rate of change of quaternion
    ahrs->quaternion = FusionQuaternionAdd(ahrs->quaternion, FusionQuaternionMultiplyVector(ahrs->quaternion, FusionVectorMultiplyScalar(adjustedHalfGyroscope, deltaTime)));

    // Normalise quaternion
    ahrs->quaternion = FusionQuaternionNormalise(ahrs->quaternion);
#undef Q
}

/**
 * @brief Updates the AHRS algorithm using the gyroscope and accelerometer
 * measurements only.
 * @param ahrs AHRS algorithm structure.
 * @param gyroscope Gyroscope measurement in degrees per second.
 * @param accelerometer Accelerometer measurement in g.
 * @param deltaTime Delta time in seconds.
 */
void FusionAhrsUpdateNoMagnetometer(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const fixed_pt_num deltaTime) {

    // Update AHRS algorithm
    FusionAhrsUpdate(ahrs, gyroscope, accelerometer, FUSION_VECTOR_ZERO, deltaTime);

    // Zero heading during initialisation
    if ((ahrs->initialising == true) && (ahrs->accelerationRejectionTimeout == false)) {
        FusionAhrsSetHeading(ahrs, fixed_pt_num{0.0});
    }
}

/**
 * @brief Updates the AHRS algorithm using the gyroscope, accelerometer, and
 * heading measurements.
 * @param ahrs AHRS algorithm structure.
 * @param gyroscope Gyroscope measurement in degrees per second.
 * @param accelerometer Accelerometer measurement in g.
 * @param heading Heading measurement in degrees.
 * @param deltaTime Delta time in seconds.
 */
void FusionAhrsUpdateExternalHeading(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const fixed_pt_num heading, const fixed_pt_num deltaTime) {
#define Q ahrs->quaternion.element

    // Calculate roll
    const fixed_pt_num roll = atan2(Q.w * Q.x + Q.y * Q.z, fixed_pt_num{0.5} - Q.y * Q.y - Q.x * Q.x);

    // Calculate magnetometer
    const fixed_pt_num headingRadians = FusionDegreesToRadians(heading);
    const fixed_pt_num sinHeadingRadians = sin(headingRadians);
    FusionVector magnetometer; // should be const
    magnetometer.axis.x = cos(headingRadians);
    magnetometer.axis.y = fixed_pt_num{-1.0} * cos(roll) * sinHeadingRadians;
    magnetometer.axis.z = sinHeadingRadians * sin(roll);

    // Update AHRS algorithm
    FusionAhrsUpdate(ahrs, gyroscope, accelerometer, magnetometer, deltaTime);
#undef Q
}

/**
 * @brief Returns the quaternion describing the sensor relative to the Earth.
 * @param ahrs AHRS algorithm structure.
 * @return Quaternion describing the sensor relative to the Earth.
 */
FusionQuaternion FusionAhrsGetQuaternion(const FusionAhrs *const ahrs) {
    return ahrs->quaternion;
}

/**
 * @brief Returns the linear acceleration measurement equal to the accelerometer
 * measurement with the 1 g of gravity removed.
 * @param ahrs AHRS algorithm structure.
 * @return Linear acceleration measurement in g.
 */
FusionVector FusionAhrsGetLinearAcceleration(const FusionAhrs *const ahrs) {
#define Q ahrs->quaternion.element
    FusionVector gravity; // should be const
    gravity.axis.x = fixed_pt_num{2.0} * (Q.x * Q.z - Q.w * Q.y);
    gravity.axis.y = fixed_pt_num{2.0} * (Q.y * Q.z + Q.w * Q.x);
    gravity.axis.z = fixed_pt_num{2.0} * (Q.w * Q.w - fixed_pt_num{0.5} + Q.z * Q.z);
    // third column of transposed rotation matrix
    const FusionVector linearAcceleration = FusionVectorSubtract(ahrs->accelerometer, gravity);
    return linearAcceleration;
#undef Q
}

/**
 * @brief Returns the Earth acceleration measurement equal to accelerometer
 * measurement in the Earth coordinate frame with the 1 g of gravity removed.
 * @param ahrs AHRS algorithm structure.
 * @return Earth acceleration measurement in g.
 */
FusionVector FusionAhrsGetEarthAcceleration(const FusionAhrs *const ahrs) {
#define Q ahrs->quaternion.element
#define A ahrs->accelerometer.axis
    const fixed_pt_num qwqw = Q.w * Q.w; // calculate common terms to avoid repeated operations
    const fixed_pt_num qwqx = Q.w * Q.x;
    const fixed_pt_num qwqy = Q.w * Q.y;
    const fixed_pt_num qwqz = Q.w * Q.z;
    const fixed_pt_num qxqy = Q.x * Q.y;
    const fixed_pt_num qxqz = Q.x * Q.z;
    const fixed_pt_num qyqz = Q.y * Q.z;
    FusionVector earthAcceleration; // should be const
    earthAcceleration.axis.x = fixed_pt_num{2.0} * ((qwqw - fixed_pt_num{0.5} + Q.x * Q.x) * A.x + (qxqy - qwqz) * A.y + (qxqz + qwqy) * A.z);
    earthAcceleration.axis.y = fixed_pt_num{2.0} * ((qxqy + qwqz) * A.x + (qwqw - fixed_pt_num{0.5} + Q.y * Q.y) * A.y + (qyqz - qwqx) * A.z);
    earthAcceleration.axis.z = (fixed_pt_num{2.0} * ((qxqz - qwqy) * A.x + (qyqz + qwqx) * A.y + (qwqw - fixed_pt_num{0.5} + Q.z * Q.z) * A.z)) - fixed_pt_num{1.0};
    // rotation matrix multiplied with the accelerometer, with 1 g subtracted
    return earthAcceleration;
#undef Q
#undef A
}

/**
 * @brief Returns the AHRS algorithm internal states.
 * @param ahrs AHRS algorithm structure.
 * @return AHRS algorithm internal states.
 */
FusionAhrsInternalStates FusionAhrsGetInternalStates(const FusionAhrs *const ahrs) {
    const FusionAhrsInternalStates internalStates = {
            .accelerationError = FusionRadiansToDegrees(FusionAsin(fixed_pt_num{2.0} * FusionVectorMagnitude(ahrs->halfAccelerometerFeedback))),
            .accelerometerIgnored = ahrs->accelerometerIgnored,
            .accelerationRejectionTimer = ahrs->settings.rejectionTimeout == 0 ? fixed_pt_num{0.0} : (fixed_pt_num) ahrs->accelerationRejectionTimer / (fixed_pt_num) ahrs->settings.rejectionTimeout,
            .magneticError = FusionRadiansToDegrees(FusionAsin(fixed_pt_num{2.0} * FusionVectorMagnitude(ahrs->halfMagnetometerFeedback))),
            .magnetometerIgnored = ahrs->magnetometerIgnored,
            .magneticRejectionTimer = ahrs->settings.rejectionTimeout == 0 ? fixed_pt_num{0.0} : (fixed_pt_num) ahrs->magneticRejectionTimer / (fixed_pt_num) ahrs->settings.rejectionTimeout,
    };
    return internalStates;
}

/**
 * @brief Returns the AHRS algorithm flags.
 * @param ahrs AHRS algorithm structure.
 * @return AHRS algorithm flags.
 */
FusionAhrsFlags FusionAhrsGetFlags(FusionAhrs *const ahrs) {
    const unsigned int warningTimeout = ahrs->settings.rejectionTimeout / 4;
    const FusionAhrsFlags flags = {
            .initialising = ahrs->initialising,
            .accelerationRejectionWarning = ahrs->accelerationRejectionTimer > warningTimeout,
            .accelerationRejectionTimeout = ahrs->accelerationRejectionTimeout,
            .magneticRejectionWarning = ahrs->magneticRejectionTimer > warningTimeout,
            .magneticRejectionTimeout = ahrs->magneticRejectionTimeout,
    };
    return flags;
}

/**
 * @brief Sets the heading of the orientation measurement provided by the AHRS
 * algorithm.  This function can be used to reset drift in heading when the AHRS
 * algorithm is being used without a magnetometer.
 * @param ahrs AHRS algorithm structure.
 * @param heading Heading angle in degrees.
 */
void FusionAhrsSetHeading(FusionAhrs *const ahrs, const fixed_pt_num heading) {
#define Q ahrs->quaternion.element
    const fixed_pt_num yaw = atan2(Q.w * Q.z + Q.x * Q.y, fixed_pt_num{0.5} - Q.y * Q.y - Q.z * Q.z);
    const fixed_pt_num halfYawMinusHeading = fixed_pt_num{2.0} * (yaw - FusionDegreesToRadians(heading));
    FusionQuaternion rotation;
    rotation.element.w = cos(halfYawMinusHeading);
    rotation.element.x = fixed_pt_num{0.0};
    rotation.element.y = fixed_pt_num{0.0};
    rotation.element.z = fixed_pt_num{-1.0} * sin(halfYawMinusHeading);
    ahrs->quaternion = FusionQuaternionMultiply(rotation, ahrs->quaternion);
#undef Q
}

//------------------------------------------------------------------------------
// End of file
