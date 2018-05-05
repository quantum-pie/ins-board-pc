/*
 * positionsim.cpp
 *
 *      Author: Ermakov_P
 */
#include "positionsim.h"
#include "geometry.h"

PositionSim::PositionSim(const FilterParams & par)
	: is_initialized(false),
	  params {par},
	  x { state_type::Zero() },
	  earth_model{ Ellipsoid::sphere }
{}

PositionSim::~PositionSim() = default;

const Ellipsoid & PositionSim::get_ellipsoid() const
{
    return earth_model.get_ellipsoid();
}

void PositionSim::step(const FilterInput & z)
{
	if(is_initialized)
	{
		step_initialized(z);
	}
	else
	{
		step_uninitialized(z);
	}
}

void PositionSim::reset()
{
	is_initialized = false;
}

void PositionSim::step_uninitialized(const FilterInput & z)
{
    x.segment<3>(0) = geom::geodetic_to_cartesian(z.geo, get_ellipsoid());

    double vn = std::cos(params.initial_track);
    double ve = std::sin(params.initial_track);
    double vu = 0;

    Vector3D enu;
    enu << ve, vn, vu;

    x.segment<3>(3) = geom::enu_to_ecef(enu, z.geo);
    x.segment<3>(6) = Vector3D::Zero();

    is_initialized = true;
}

void PositionSim::step_initialized(const FilterInput & z)
{
    const Ellipsoid & el = get_ellipsoid();

    Vector3D geo = geom::cartesian_to_geodetic(get_cartesian(), el);

    double distance = params.speed * z.dt;

    Vector3D local_vel = geom::geodetic_to_dcm(geo) * get_velocity();
    double bearing = std::atan2(local_vel[0], local_vel[1]);

    geo = geom::great_circle_destination(geo, bearing, distance, el);

    Vector3D new_pos = geom::geodetic_to_cartesian(geo, el);
    Vector3D new_speed = (new_pos - get_cartesian()) / z.dt;

    x.segment<3>(6) = (new_speed - get_velocity()) / z.dt;
    x.segment<3>(3) = new_speed;
    x.segment<3>(0) = new_pos;
}

Vector3D PositionSim::get_cartesian() const
{
    return x.segment<3>(0);
}

Vector3D PositionSim::get_velocity() const
{
    return x.segment<3>(3);
}

Vector3D PositionSim::get_acceleration() const
{
    return x.segment<3>(6);
}

void PositionSim::set_initial_track(double radians)
{
	params.initial_track = radians;
}

void PositionSim::set_speed(double ms)
{
	params.speed = ms;
}

double PositionSim::get_initial_track() const
{
	return params.initial_track;
}

double PositionSim::get_speed() const
{
	return params.speed;
}
