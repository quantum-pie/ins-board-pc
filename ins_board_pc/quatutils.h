/*
 * quatutils.h
 *
 *      Author: Ermakov_P
 */

#ifndef INCLUDE_QUATUTILS_H_
#define INCLUDE_QUATUTILS_H_

#include "quaternion.h"

namespace quat
{

/*!
 * @brief Create quaternion which describes rotation of vector by given angle around x axis.
 * @param radians rotation angle.
 * @return rotation quaternion.
 */
Quaternion x_rotator(double radians);

/*!
 * @brief Create quaternion which describes rotation of vector by given angle around y axis.
 * @param radians rotation angle.
 * @return rotation quaternion.
 */
Quaternion y_rotator(double radians);

/*!
 * @brief Create quaternion which describes rotation of vector by given angle around z axis.
 * @param radians rotation angle.
 * @return rotation quaternion.
 */
Quaternion z_rotator(double radians);

/*!
 * @brief Get skew-symmetric matrix corresponding to given vector.
 * @param v vector.
 * @return skew-symmetric matrix.
 */
Quaternion::skew_type skew_symmetric(const Vector3D & v);

/*!
 * @brief Quaternion scalar product operator.
 * @param v scalar operand.
 * @param q quaternion operand.
 * @return product of this v with p.
 */
Quaternion operator*(double v, const Quaternion & q);

/*!
 * @brief Simple quaternion linear interpolation.
 * @param q first operand.
 * @param p second operand.
 * @param alpha first operand weight.
 * @return interpolated quaternion.
 */
Quaternion lerp(const Quaternion & q, const Quaternion & p, double alpha);

/*!
 * @brief Spherical quaternion linear interpolation.
 * @param q first operand.
 * @param p second operand.
 * @param alpha first operand weight.
 * @return interpolated quaternion.
 */
Quaternion slerp(const Quaternion & q, const Quaternion & p, double alpha);

}

#endif /* INCLUDE_QUATUTILS_H_ */
