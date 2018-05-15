/*
 * positionsim.cpp
 *
 *      Author: Ermakov_P
 */
#include "positionsim.h"
#include "ellipsoid.h"
#include "packets.h"
#include "geometry.h"

struct PositionSim::Impl
{
    Impl(const PositionSim & parent, const FilterParams & params)
        : p{ parent },
          is_initialized{ false },
          params{ params },
          x { state_type::Zero() }
    {}

    /*!
     * @brief Step of uninitialized filter.
     * @param z filter input.
     */
    void step_uninitialized(const FilterInput & z)
    {
        x.segment<3>(0) = geom::geodetic_to_cartesian(z.geo, p.get_ellipsoid());

        double vn = std::cos(params.initial_track);
        double ve = std::sin(params.initial_track);
        double vu = 0;

        Vector3D enu;
        enu << ve, vn, vu;

        x.segment<3>(3) = geom::enu_to_ecef(enu, z.geo);
        x.segment<3>(6) = Vector3D::Zero();

        is_initialized = true;
    }

    /*!
     * @brief Step of initialized filter.
     * @param z filter input.
     */
    void step_initialized(const FilterInput & z)
    {
        const Ellipsoid & el = p.get_ellipsoid();

        Vector3D geo = geom::cartesian_to_geodetic(p.get_cartesian(), el);

        double distance = params.speed * z.dt;

        Vector3D local_vel = geom::geodetic_to_dcm(geo) * p.get_velocity();
        double bearing = std::atan2(local_vel[0], local_vel[1]);

        geo = geom::great_circle_destination(geo, bearing, distance, el);

        Vector3D new_pos = geom::geodetic_to_cartesian(geo, el);
        Vector3D new_speed = (new_pos - p.get_cartesian()) / z.dt;

        x.segment<3>(6) = (new_speed - p.get_velocity()) / z.dt;
        x.segment<3>(3) = new_speed;
        x.segment<3>(0) = new_pos;
    }

    static constexpr int state_size { 9 };        	//!< Size of state vector.
    using state_type = StaticVector<state_size>;

    const PositionSim & p;
    bool is_initialized;                            //!< Filter is initialized flag.
    FilterParams params;                            //!< Filter parameters instance.
    state_type x;                                   //!< Filter state.
};


PositionSim::PositionSim(const FilterParams & par)
    : pimpl{ std::make_unique<Impl>(*this, par) }
{}

PositionSim::~PositionSim() = default;

void PositionSim::set_initial_track(double radians)
{
    pimpl->params.initial_track = radians;
}

void PositionSim::set_speed(double ms)
{
    pimpl->params.speed = ms;
}

double PositionSim::get_initial_track() const
{
    return pimpl->params.initial_track;
}

double PositionSim::get_speed() const
{
    return pimpl->params.speed;
}

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
    return pimpl->x.segment<3>(0);
}

Ellipsoid PositionSim::do_get_ellipsoid() const
{
    return Ellipsoid::sphere;
}

Vector3D PositionSim::do_get_velocity() const
{
    return pimpl->x.segment<3>(3);
}

Vector3D PositionSim::do_get_acceleration() const
{
    return pimpl->x.segment<3>(6);
}
