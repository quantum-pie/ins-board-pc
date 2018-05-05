/*
 * positionbypass.cpp
 *
 *      Author: bigaw
 */
#include "positionbypass.h"

PositionBypass::PositionBypass()
	: x{ state_type::Zero() }
{}

PositionBypass::~PositionBypass() = default;

const Ellipsoid & PositionBypass::get_ellipsoid() const
{
    return Ellipsoid::WGS84;
}

void PositionBypass::step(const FilterInput & z)
{
	x.segment<3>(0) = z.pos;
	x.segment<3>(3) = z.v;
	x.segment<3>(6) = z.geo;
}

void PositionBypass::reset() {}

Vector3D PositionBypass::get_cartesian() const
{
    return x.segment<3>(0);
}

Vector3D PositionBypass::get_velocity() const
{
    return x.segment<3>(3);
}

Vector3D PositionBypass::get_acceleration() const
{
    return Vector3D::Zero();
}
