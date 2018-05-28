/*
 * utils.h
 *
 *      Author: Ermakov_P
 */

#ifndef INCLUDE_UTILS_H_
#define INCLUDE_UTILS_H_

#include "eigenaux.h"

#include <cstdint>
#include <string>

class FilterInput;
class RawPacket;
class MagnCalibrator;
class QCustomPlot;
class QString;

/*!
 * @brief This namespace holds various general purpose utilities.
 */
namespace utils
{

/*!
 * @brief Convert angle from radians to degrees.
 * @param radians angle in radians.
 * @return angle in degrees.
 */
double radians_to_degrees(double radians);

/*!
 * @brief Convert angle from degrees to radians.
 * @param degrees angle in degrees.
 * @return angle in radians.
 */
double degrees_to_radians(double degrees);

/*!
 * @brief Bring angle in radians to +-pi range.
 * @param radians angle in radiance.
 * @return fixed angle in radians.
 */
double fix_angle(double radians);

/*!
 * @brief Convert floating-point representation to fixed-point.
 * @param val floating-point value.
 * @return fixed-point value.
 */
int32_t float_to_fixed(float val);

/*!
 * @brief Convert double precision floating-point representation to fixed-point.
 * @param val double precision floating-point value.
 * @return fixed-point value.
 */
int32_t double_to_fixed(double val);

/*!
 * @brief Convert floating-point angle in radians to fixed-point representation.
 * @param radians floating-point angle in radians.
 * @return fixed-point angle.
 */
int32_t angle_to_fixed(double radians);

/*!
 * @brief Convert speed from m/s to knots units.
 * @param ms speed in m/s.
 * @return speed in knots.
 */
double ms_to_knots(double ms);

void update_3axis_plot(QCustomPlot * plot, const Vector3D & vec);

void clear_3axis_plot(QCustomPlot * plot);

QString double_view(double val, std::size_t digits = 2);

}

#endif /* INCLUDE_UTILS_H_ */
