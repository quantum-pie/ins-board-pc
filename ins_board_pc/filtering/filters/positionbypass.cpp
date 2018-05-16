/*
 * positionbypass.cpp
 *
 *      Author: bigaw
 */
#include "positionbypass.h"
#include "packets.h"
#include "ellipsoid.h"

PositionBypass::PositionBypass()
    : pos{ Vector3D::Zero() },
      vel{ Vector3D::Zero() }
{}

void PositionBypass::do_step(const FilterInput & z)
{
    pos = z.pos;
    vel = z.v;
}

void PositionBypass::do_reset() {}

Vector3D PositionBypass::do_get_cartesian() const
{
    return pos;
}

Ellipsoid PositionBypass::do_get_ellipsoid() const
{
    return Ellipsoid::WGS84;
}

Vector3D PositionBypass::do_get_velocity() const
{
    return vel;
}

Vector3D PositionBypass::do_get_acceleration() const
{
    return Vector3D::Zero();
}
