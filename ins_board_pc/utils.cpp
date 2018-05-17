/*
 * utils.cpp
 *
 *      Author: Ermakov_P
 */
#include "utils.h"

#include <cmath>
#include <limits>

namespace utils
{

//! Multiplier to convert speed units from m/s to knots.
static constexpr double ms2knots { 1.94384 };

//! How many degrees in PI radians.
static constexpr double degrees_per_pi { 180.0 };

//! Multiplier to convert floating-point value into 16.16 fixed-point value.
static constexpr int16_t fixed16d16_mult { std::numeric_limits<int16_t>::max() };

double radians_to_degrees(double radians)
{
    return radians / M_PI * degrees_per_pi;
}

double degrees_to_radians(double degrees)
{
    return degrees / degrees_per_pi * M_PI;
}

double fix_angle(double radians)
{
	if(radians >= M_PI)
	{
		return radians - 2 * M_PI;
	}
	else if(radians < - M_PI)
	{
		return radians + 2 * M_PI;
	}
	else
	{
		return radians;
	}
}

int32_t float_to_fixed(float val)
{
    return static_cast<int32_t>(val * fixed16d16_mult);
}

int32_t double_to_fixed(double val)
{
    return static_cast<int32_t>(val * fixed16d16_mult);
}

int32_t angle_to_fixed(double radians)
{
    return static_cast<int32_t>(radians * fixed16d16_mult / M_PI);
}

double ms_to_knots(double ms)
{
	return ms * ms2knots;
}

}


