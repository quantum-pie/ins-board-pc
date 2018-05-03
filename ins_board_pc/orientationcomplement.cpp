#include "orientationcomplement.h"

#include <QtMath>
#include <QDebug>

const int OrientationCF::state_size = 7;

OrientationCF::OrientationCF(const FilterParams & par)
    : OrientationFilter(par.accum_capacity), params(par)
{
    x = NumVector(state_size);
}

OrientationCF::~OrientationCF()
{

}

NumVector OrientationCF::measured_quaternion(const NumVector & accel, const NumVector & magn) const
{
    NumVector qacc = qutils::acceleration_quat(accel);
    NumMatrix accel_rotator = qutils::quaternion_to_dcm_tr(qacc);
    NumVector l = accel_rotator * magn;
    NumVector qmag = qutils::magnetometer_quat(l);
    return qutils::quat_multiply(qacc, qmag);
}

void OrientationCF::initialize(const FilterInput & z)
{
    OrientationFilter::initialize(z);
    x.segment(0, 4) = qutils::quat_conjugate(measured_quaternion(z.a, z.m));

    x[4] = bias_x_ctrl.get_mean();
    x[5] = bias_y_ctrl.get_mean();
    x[6] = bias_z_ctrl.get_mean();
}

void OrientationCF::step(const FilterInput & z)
{
    if(is_initialized())
    {
        update(z);
    }
    else if(bias_estimated())
    {
        initialize(z);
    }
    else
    {
        accumulate(z);
    }
}

void OrientationCF::update(const FilterInput & z)
{
    /* useful constants */
    double dt_2 = z.dt / 2;

    /* constructing the quaternion propagation matrix */
    NumMatrix V = qutils::skew_symmetric(z.w);

    V *= dt_2;
    V += NumMatrix::Identity(4, 4);

    NumMatrix K = qutils::quat_delta_mtx(get_orientation_quaternion(), dt_2);

    NumMatrix F(state_size, state_size);

    F <<  V, K,
          NumMatrix::Zero(3, 4), NumMatrix::Identity(3, 3);

    /* residual quaternion */
    NumVector qerr = qutils::quat_multiply(measured_quaternion(z.a, z.m), get_orientation_quaternion());

    /* error angles */
    NumVector rpy_err = qutils::quat_to_rpy(qerr);

    /* update bias */
    x[4] += params.bias_gain * rpy_err[1];
    x[5] += params.bias_gain * rpy_err[0];
    x[6] += -params.bias_gain * rpy_err[2];

    /* propagate quaternion */
    x = F * x;
    normalize_state();

    NumVector ident_quat = qutils::identity_quaternion();

    /* extract predicted quaternion */
    NumVector q_pred = get_orientation_quaternion();

    NumVector a_norm = z.a / z.a.norm();

    /* predict gravity */
    NumVector g_pred = qutils::quaternion_to_dcm(q_pred) * a_norm;
    NumVector qacc_delta = qutils::quat_conjugate(qutils::acceleration_quat(g_pred));
    NumVector qacc_corr = qutils::lerp(ident_quat, qacc_delta, calculate_gain(z.a));
    NumVector q_corr = qutils::quat_multiply(qacc_corr, q_pred);

    /* predict magnetic vector */
    NumVector mag_pred = qutils::quaternion_to_dcm(q_corr) * z.m;
    NumVector qmag_delta = qutils::quat_conjugate(qutils::magnetometer_quat(mag_pred));
    NumVector qmag_corr = qutils::lerp(ident_quat, qmag_delta, params.static_magn_gain);
    q_corr = qutils::quat_multiply(qmag_corr, q_corr);

    /* finish */
    x.segment(0, 4) = q_corr;
}

void OrientationCF::normalize_state()
{
    x.segment(0, 4) = qutils::quat_normalize(get_orientation_quaternion());
}

double OrientationCF::calculate_gain(const NumVector & accel) const
{
    // TODO
    return params.static_accel_gain;
}

NumVector OrientationCF::get_orientation_quaternion() const
{
    return x.segment(0, 4);
}

NumVector OrientationCF::get_gyro_bias() const
{
    return x.segment(4, 3);
}

void OrientationCF::set_static_accel_gain(double gain)
{
    params.static_accel_gain = gain;
}

void OrientationCF::set_static_magn_gain(double gain)
{
    params.static_magn_gain = gain;
}
