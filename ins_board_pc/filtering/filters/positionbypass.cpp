/*
 * positionbypass.cpp
 *
 *      Author: bigaw
 */
#include "positionbypass.h"
#include "packets.h"
#include "ellipsoid.h"

struct PositionBypass::Impl
{
    Impl() : x{ state_type::Zero() } {}

    static constexpr int state_size { 9 };        	//!< Size of state vector.
    using state_type = StaticVector<state_size>;
    state_type x;									//!< State vector.
};

PositionBypass::PositionBypass()
    : pimpl{ std::make_unique<Impl>() }
{}

PositionBypass::~PositionBypass() = default;

void PositionBypass::do_step(const FilterInput & z)
{
    pimpl->x.segment<3>(0) = z.pos;
    pimpl->x.segment<3>(3) = z.v;
    pimpl->x.segment<3>(6) = z.geo;
}

void PositionBypass::do_reset() {}

Vector3D PositionBypass::do_get_cartesian() const
{
    return pimpl->x.segment<3>(0);
}

Ellipsoid PositionBypass::do_get_ellipsoid() const
{
    return Ellipsoid::WGS84;
}

Vector3D PositionBypass::do_get_velocity() const
{
    return pimpl->x.segment<3>(3);
}

Vector3D PositionBypass::do_get_acceleration() const
{
    return Vector3D::Zero();
}
