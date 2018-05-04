#include "orientationcomplement.h"
#include "quatutils.h"
#include "geometry.h"

using namespace quat;
using namespace geom;

OrientationCF::OrientationCF(const FilterParams & par)
    : is_initialized {false},
	  params{par},
	  bias_ctrl{ par.accum_capacity, Vector3D::Zero() },
	  x { state_type::Zero() }
{}

OrientationCF::~OrientationCF() = default;

OrientationCF::FInput OrientationCF::adopt_input(const FilterInput & z)
{
	Vector3D m_corr = z_rotator(earth_model.magnetic_declination(z.geo, z.day)).dcm_tr() * z.m;
	return {z.w, z.a, m_corr, z.dt};
}

void OrientationCF::reset()
{
	is_initialized = false;
}

void OrientationCF::step(const FilterInput & z)
{
	FInput zc = adopt_input(z);

    if(is_initialized)
    {
        step_initialized(zc);
    }
    else if(bias_ctrl.is_saturated())
    {
        initialize(zc);
    }
    else
    {
    	step_uninitialized(zc);
    }
}

void OrientationCF::initialize(const FInput & z)
{
    x.segment<4>(0) = static_cast<Quaternion::vector_form>(accel_magn_quat(z.a, z.m).conjugate());
    x.segment<3>(4) = bias_ctrl.get_mean();

    is_initialized = true;
    bias_ctrl.set_sampling(0); // free memory
}

void OrientationCF::step_uninitialized(const FInput & z)
{
    bias_ctrl.update(z.w);
}

void OrientationCF::step_initialized(const FInput & z)
{
    /* useful constants */
    const double dt_2 = z.dt / 2;

    /* constructing the quaternion propagation matrix */
    auto V = quat::skew_symmetric(z.w);

    V *= dt_2;
    V += V_type::Identity();

    auto K = get_orientation_quaternion().delta_mtx(dt_2);

    F_type F;

    F <<  V, K,
          StaticMatrix<3, 4>::Zero(), StaticMatrix<3, 3>::Identity();

	/* residual quaternion */
	Quaternion qerr = accel_magn_quat(z.a, z.m) *  get_orientation_quaternion();

	/* error angles */
	Vector3D rpy_err = qerr.rpy();

	/* update bias */
	x[4] += params.bias_gain * rpy_err[1];
	x[5] += params.bias_gain * rpy_err[0];
	x[6] += -params.bias_gain * rpy_err[2];

    /* propagate quaternion */
    x = F * x;
    normalize_state();

    /* extract predicted quaternion */
    Quaternion q_pred = get_orientation_quaternion();

    Vector3D a_norm = z.a / z.a.norm();

    /* predict gravity */
    Vector3D g_pred = q_pred.dcm() * a_norm;
    Quaternion qacc_delta = acceleration_quat(g_pred).conjugate();
    Quaternion qacc_corr = lerp(Quaternion::identity, qacc_delta, calculate_gain(z.a));
    Quaternion q_corr = qacc_corr * q_pred;

    /* predict magnetic vector */
    Vector3D mag_pred = q_corr.dcm() * z.m;
    Quaternion qmag_delta = magnetometer_quat(mag_pred).conjugate();
    Quaternion qmag_corr = lerp(Quaternion::identity, qmag_delta, params.static_magn_gain);
    q_corr = qmag_corr * q_corr;

    /* finish */
    x.segment<4>(0) = static_cast<Quaternion::vector_form>(q_corr);
}

void OrientationCF::normalize_state()
{
    x.segment<4>(0) = static_cast<Quaternion::vector_form>(get_orientation_quaternion().normalize());
}

double OrientationCF::calculate_gain(const Vector3D & accel) const
{
    // TODO
    return params.static_accel_gain;
}

Quaternion OrientationCF::get_orientation_quaternion() const
{
    return static_cast<Quaternion::vector_form>(x.segment<4>(0));
}

Vector3D OrientationCF::get_gyro_bias() const
{
    return x.segment<3>(4);
}

void OrientationCF::set_static_accel_gain(double gain)
{
    params.static_accel_gain = gain;
}

void OrientationCF::set_static_magn_gain(double gain)
{
    params.static_magn_gain = gain;
}

void OrientationCF::set_bias_gain(double gain)
{
	params.bias_gain = gain;
}

double OrientationCF::get_static_accel_gain() const
{
	return params.static_accel_gain;
}

double OrientationCF::get_static_magn_gain() const
{
	return params.static_magn_gain;
}

double OrientationCF::get_bias_gain() const
{
	return params.bias_gain;
}
