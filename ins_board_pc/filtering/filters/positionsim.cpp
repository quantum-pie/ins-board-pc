/*
 * positionsim.cpp
 *
 *      Author: Ermakov_P
 */
#include "eigenaux.h"
#include "positionsim.h"
#include "ellipsoid.h"
#include "packets.h"
#include "geometry.h"

struct PositionSim::Impl
{
    Impl()
        : is_initialized{ false },
          pos{ Vector3D::Zero() },
          vel{ Vector3D::Zero() },
          acc{ Vector3D::Zero() },
          params{ default_params }
    {}

    void step_uninitialized(const FilterInput & z)
    {
        pos = geom::geodetic_to_cartesian(z.geo, el);

        double vn = std::cos(params.initial_track);
        double ve = std::sin(params.initial_track);
        double vu = 0;

        Vector3D enu;
        enu << ve, vn, vu;

        vel = geom::enu_to_ecef(enu, z.geo);
        acc = Vector3D::Zero();

        is_initialized = true;
    }

    void step_initialized(const FilterInput & z)
    {
        Vector3D geo = geom::cartesian_to_geodetic(pos, el);

        double distance = params.speed * z.dt;

        Vector3D local_vel = geom::geodetic_to_dcm(geo) * vel;
        double bearing = std::atan2(local_vel[0], local_vel[1]);

        geo = geom::great_circle_destination(geo, bearing, distance, el);

        Vector3D new_pos = geom::geodetic_to_cartesian(geo, el);
        Vector3D new_speed = (new_pos - pos) / z.dt;

        acc = (new_speed - vel) / z.dt;
        vel = new_speed;
        pos = new_pos;
    }

    bool is_initialized;

    Vector3D pos;
    Vector3D vel;
    Vector3D acc;

    struct FilterParams
    {
        double initial_track;       //!< Start track angle.
        double speed;               //!< Movement speed.
    } params;

    static const Ellipsoid & el;
    static constexpr FilterParams default_params { 0, 30 };
};

const Ellipsoid & PositionSim::Impl::el { Ellipsoid::sphere };

PositionSim::PositionSim()
    : pimpl{ std::make_unique<Impl>() }
{}

PositionSim::~PositionSim() = default;

void PositionSim::do_step(const FilterInput & z)
{
    if(pimpl->is_initialized)
    {
        pimpl->step_initialized(z);
    }
    else
    {
        pimpl->step_uninitialized(z);
    }
}

void PositionSim::do_reset()
{
    pimpl->is_initialized = false;
}

Vector3D PositionSim::do_get_cartesian() const
{
    return pimpl->pos;
}

Ellipsoid PositionSim::do_get_ellipsoid() const
{
    return pimpl->el;
}

Vector3D PositionSim::do_get_velocity() const
{
    return pimpl->vel;
}

Vector3D PositionSim::do_get_acceleration() const
{
    return pimpl->acc;
}

void PositionSim::do_set_initial_track(double radians)
{
    pimpl->params.initial_track = radians;
}

void PositionSim::do_set_speed(double ms)
{
    pimpl->params.speed = ms;
}

double PositionSim::do_get_initial_track() const
{
    return pimpl->params.initial_track;
}

double PositionSim::do_get_speed() const
{
    return pimpl->params.speed;
}
