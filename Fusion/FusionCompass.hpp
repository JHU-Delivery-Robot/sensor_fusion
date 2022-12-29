/**
 * @file FusionCompass.h
 * @author Seb Madgwick
 * @brief Tilt-compensated compass to calculate an heading relative to magnetic
 * north using accelerometer and magnetometer measurements.
 */

#ifndef FUSION_COMPASS_HPP
#define FUSION_COMPASS_HPP

//------------------------------------------------------------------------------
// Includes

#include "FusionMath.hpp"

//------------------------------------------------------------------------------
// Function declarations

fixed_pt_num FusionCompassCalculateHeading(const FusionVector accelerometer, const FusionVector magnetometer);

#endif

//------------------------------------------------------------------------------
// End of file
