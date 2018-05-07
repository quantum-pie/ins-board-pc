/*
 * positionlkf.cpp
 *
 *      Author: Ermakov_P
 */
#include "positionlkf.h"
#include "geometry.h"

#include <Eigen/Dense>

PositionLKF::PositionLKF(const FilterParams & par)
	: is_initialized(false),
	  params {par}
{
	x = state_type::Zero();
	P = P_type::Identity();

    local_cov = create_local_cov_mtx();
}

PositionLKF::~PositionLKF() = default;

Ellipsoid PositionLKF::get_ellipsoid() const
{
    return Ellipsoid::WGS84;
}

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

void PositionLKF::step_uninitialized(const FilterInput & z)
{
	x.segment<3>(0) = z.pos;
	x.segment<3>(3) = z.v;
	x.segment<3>(6) = Vector3D::Zero();

	auto diag = P.diagonal();
	diag.segment<3>(0) = Vector3D::Constant(params.init_params.pos_std * params.init_params.pos_std);
	diag.segment<3>(3) = Vector3D::Constant(params.init_params.vel_std * params.init_params.vel_std);
	diag.segment<3>(6) = Vector3D::Constant(params.init_params.accel_std * params.init_params.accel_std);

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

PositionLKF::F_type PositionLKF::create_transition_mtx(double dt) const
{
    /* useful constants */
    const double dt_sq = dt * dt;
    const double dt_sq_2 = dt_sq / 2;

    /* constructing state transition matrix */
    F_type F;

    const auto I3 = Matrix3D::Identity();

    F << I3, dt * I3, dt_sq_2 * I3,
    	 StaticMatrix<3, 3>::Zero(), I3, dt * I3,
		 StaticMatrix<3, 6>::Zero(), I3;

    return F;
}

PositionLKF::Q_type PositionLKF::create_proc_noise_cov_mtx(double dt) const
{
    /* useful constants */
    double dt_sq = dt * dt;
    double dt_sq_2 = dt_sq / 2;

    G_type G;

    const auto I3 = Matrix3D::Identity();

    G << I3 * dt_sq_2,
    	 I3 * dt,
		 I3;

    return params.proc_params.accel_std * params.proc_params.accel_std * G * G.transpose();
}

Matrix3D PositionLKF::create_local_cov_mtx() const
{
    const double horizontal_linear_std = params.meas_params.gps_cep * 1.2;
    const double altitude_std = horizontal_linear_std / 0.53;

    Matrix3D local_cov = Matrix3D::Zero();
    local_cov(0, 0) = horizontal_linear_std * horizontal_linear_std;
    local_cov(1, 1) = horizontal_linear_std * horizontal_linear_std;
    local_cov(2, 2) = altitude_std * altitude_std;

    return local_cov;
}

PositionLKF::R_type PositionLKF::create_meas_noise_cov_mtx(const Vector3D & geo) const
{
    Matrix3D Cel = geom::geodetic_to_dcm(geo);
    Matrix3D Rp = Cel.transpose() * local_cov * Cel;

    R_type R;

    const double vel_variance = params.meas_params.gps_vel_std * params.meas_params.gps_vel_std;

    R << Rp, Matrix3D::Zero(),
    	 Matrix3D::Zero(), Matrix3D::Identity() * vel_variance;

    return R;
}

PositionLKF::H_type PositionLKF::create_meas_proj_mtx() const
{
    H_type H;

    H <<   Matrix3D::Identity(), StaticMatrix<3, 6>::Zero(),
    		Matrix3D::Zero(), Matrix3D::Identity(), Matrix3D::Zero();

    return H;
}

Vector3D PositionLKF::get_cartesian() const
{
    return x.segment<3>(0);
}

Vector3D PositionLKF::get_velocity() const
{
    return x.segment<3>(3);
}

Vector3D PositionLKF::get_acceleration() const
{
    return x.segment<3>(6);
}

void PositionLKF::set_proc_accel_std(double std)
{
    params.proc_params.accel_std = std;
}

void PositionLKF::set_meas_pos_std(double std)
{
    params.meas_params.gps_cep = std;
}

void PositionLKF::set_meas_vel_std(double std)
{
    params.meas_params.gps_vel_std = std;
}

void PositionLKF::set_init_pos_std(double std)
{
    params.init_params.pos_std = std;
}

void PositionLKF::set_init_vel_std(double std)
{
    params.init_params.vel_std = std;
}

void PositionLKF::set_init_accel_std(double std)
{
    params.init_params.accel_std = std;
}

double PositionLKF::get_proc_accel_std() const
{
	return params.proc_params.accel_std;
}

double PositionLKF::get_meas_pos_std() const
{
	return params.meas_params.gps_cep;
}

double PositionLKF::get_meas_vel_std() const
{
	return params.meas_params.gps_vel_std;
}

double PositionLKF::get_init_pos_std() const
{
	return params.init_params.pos_std;
}

double PositionLKF::get_init_vel_std() const
{
	return params.init_params.vel_std;
}

double PositionLKF::get_init_accel_std() const
{
	return params.init_params.accel_std;
}
