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

PositionLKF::PositionLKF(const FilterParams & par)
    : KalmanPositionFilterBase{ par },
      is_initialized{ false },
      x{ state_type::Zero() },
      P{ P_type::Identity() }
{}

PositionLKF::~PositionLKF() = default;

void PositionLKF::step(const FilterInput & z)
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

void PositionLKF::reset()
{
    is_initialized = false;
}

Vector3D PositionLKF::get_cartesian() const
{
    return x.segment<3>(0);
}

Ellipsoid PositionLKF::get_ellipsoid() const
{
    return Ellipsoid::WGS84;
}

Vector3D PositionLKF::get_velocity() const
{
    return x.segment<3>(3);
}

Vector3D PositionLKF::get_acceleration() const
{
    return x.segment<3>(6);
}

void PositionLKF::step_uninitialized(const FilterInput & z)
{
    x.segment<3>(0) = z.pos;
    x.segment<3>(3) = z.v;
    x.segment<3>(6) = Vector3D::Zero();

    P = create_init_cov_mtx();
    is_initialized = true;
}

void PositionLKF::step_initialized(const FilterInput & z)
{
    F_type F = create_transition_mtx(z.dt);
    Q_type Q = create_proc_noise_cov_mtx(z.dt);

    x = F * x;
    P = F * P * F.transpose() + Q;

    if(z.gps_valid)
    {
        meas_type z_pr;
        z_pr << get_cartesian(), get_velocity();

        meas_type z_meas;
        z_meas << z.pos, z.v;

        auto y = z_meas - z_pr;

        H_type H = create_meas_proj_mtx();
        R_type R = create_meas_noise_cov_mtx(geom::cartesian_to_geodetic(get_cartesian(), get_ellipsoid()));

        auto S = H * P * H.transpose() + R;
        auto K = P * H.transpose() * S.inverse();

        x += K * y;
        P = (P_type::Identity() - K * H) * P;
    }
}
