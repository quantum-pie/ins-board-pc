/*
 * positionlkf.cpp
 *
 *      Author: Ermakov_P
 */
#include "positionlkf.h"
#include "packets.h"
#include "ellipsoid.h"
#include "geometry.h"

#include <Eigen/Dense>

struct PositionLKF::Impl
{
    Impl(const PositionLKF & parent)
        : p{ parent },
          is_initialized{ false },
          x{ state_type::Zero() },
          P{ P_type::Identity() }
    {}

    /*!
     * @brief Step of initialized filter.
     * @param z filter input.
     */
    void step_initialized(const FilterInput & z)
    {
        F_type F = p.create_transition_mtx(z.dt);
        Q_type Q = p.create_proc_noise_cov_mtx(z.dt);

        x = F * x;
        P = F * P * F.transpose() + Q;

        if(z.gps_valid)
        {
            meas_type z_pr;
            z_pr << p.get_cartesian(), p.get_velocity();

            meas_type z_meas;
            z_meas << z.pos, z.v;

            auto y = z_meas - z_pr;

            H_type H = p.create_meas_proj_mtx();
            R_type R = p.create_meas_noise_cov_mtx(geom::cartesian_to_geodetic(p.get_cartesian(), p.get_ellipsoid()));

            auto S = H * P * H.transpose() + R;
            auto K = P * H.transpose() * S.inverse();

            x += K * y;
            P = (P_type::Identity() - K * H) * P;
        }
    }

    /*!
     * @brief Step of uninitialized filter.
     * @param z filter input.
     */
    void step_uninitialized(const FilterInput & z)
    {
        x.segment<3>(0) = z.pos;
        x.segment<3>(3) = z.v;
        x.segment<3>(6) = Vector3D::Zero();

        P = p.create_init_cov_mtx();
        is_initialized = true;
    }

    /* Useful aliases */
    using state_type = StaticVector<state_size>;
    using meas_type = StaticVector<measurement_size>;

    const PositionLKF & p;              //!< Parent.
    bool is_initialized;                //!< Filter is initialized flag.
    state_type x;                       //!< State vector.
    P_type P;                        	//!< State estimate covariance matrix.

};

PositionLKF::PositionLKF(const FilterParams & par)
    : KalmanPositionFilterBase{ par },
      pimpl{ std::make_unique<Impl>(*this) }
{}

PositionLKF::~PositionLKF() = default;

void PositionLKF::do_step(const FilterInput & z)
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

void PositionLKF::do_reset()
{
    pimpl->is_initialized = false;
}

Vector3D PositionLKF::do_get_cartesian() const
{
    return pimpl->x.segment<3>(0);
}

Ellipsoid PositionLKF::do_get_ellipsoid() const
{
    return Ellipsoid::WGS84;
}

Vector3D PositionLKF::do_get_velocity() const
{
    return pimpl->x.segment<3>(3);
}

Vector3D PositionLKF::do_get_acceleration() const
{
    return pimpl->x.segment<3>(6);
}
