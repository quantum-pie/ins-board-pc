/*
 * quatutils.cpp
 *
 *      Author: Ermakov_P
 */
#include "quatutils.h"

#include <cmath>

namespace quat
{

Quaternion x_rotator(double radians)
{
	return {std::cos(radians / 2), std::sin(radians / 2), 0.0, 0.0};
}

Quaternion y_rotator(double radians)
{
	return {std::cos(radians / 2), 0.0, std::sin(radians / 2), 0.0};
}

Quaternion z_rotator(double radians)
{
    return {std::cos(radians / 2), 0.0, 0.0, std::sin(radians / 2)};
}

Quaternion::skew_type skew_symmetric(const Vector3D & v)
{
	Quaternion::skew_type V;
    V <<     0,    -v[0], -v[1], -v[2],
             v[0],  0,     v[2], -v[1],
             v[1], -v[2],  0,     v[0],
             v[2],  v[1], -v[0],  0;

    return V;
}

Quaternion operator*(double v, const Quaternion & q)
{
	return q * v;
}

Quaternion lerp(const Quaternion & q, const Quaternion & p, double alpha)
{
    return (q * (1 - alpha) + p * alpha).normalize();
}

Quaternion slerp(const Quaternion & q, const Quaternion & p, double alpha)
{
    // TODO
	return lerp(q, p, alpha);
}

};



