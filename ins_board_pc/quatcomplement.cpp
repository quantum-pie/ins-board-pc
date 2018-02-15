#include "quatcomplement.h"
#include "quaternions.h"

#include <QtMath>

const int QuaternionComplement::state_size = 7;

QuaternionComplement::QuaternionComplement(const FilterParams & par)
    : AbstractOrientationFilter(par.accum_capacity), params(par)
{
    x = NumVector(state_size);
}

QuaternionComplement::~QuaternionComplement()
{

}

void QuaternionComplement::initialize(const FilterInput & z)
{
    AbstractOrientationFilter::initialize(z);

    NumVector qacc = qutils::acceleration_quat(z.a);

    NumMatrix accel_rotator = qutils::quaternion_to_dcm_tr(qacc);
    NumVector l = accel_rotator * z.m;

    NumVector qmag = qutils::magnetometer_quat(l);

    NumVector q = qutils::quat_multiply(qacc, qmag);

    // from lb to bl quaternion
    x[0] = q[0];
    x[1] = -q[1];
    x[2] = -q[2];
    x[3] = -q[3];

    x[4] = bias_x_ctrl.get_mean();
    x[5] = bias_y_ctrl.get_mean();
    x[6] = bias_z_ctrl.get_mean();
}

void QuaternionComplement::step(const FilterInput & z)
{
    accumulate(z);
    if(is_initialized())
    {
        update(z);
    }
    else if(bias_estimated())
    {
        initialize(z);
    }
}

void QuaternionComplement::update(const FilterInput & z)
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

    /* propagate quaternion */
    x = F * x;

    /* update biases */
    //x[4] = bias_x_ctrl.get_mean();
    //x[5] = bias_y_ctrl.get_mean();
    //x[6] = bias_z_ctrl.get_mean();

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
    x[0] = q_corr[0];
    x[1] = q_corr[1];
    x[2] = q_corr[2];
    x[3] = q_corr[3];
}

void QuaternionComplement::normalize_state()
{
    x.segment(0, 4) = qutils::quat_normalize(get_orientation_quaternion());
}

double QuaternionComplement::calculate_gain(const NumVector & accel) const
{
    // TODO
    return params.static_accel_gain;
}

NumVector QuaternionComplement::get_orientation_quaternion() const
{
    return x.segment(0, 4);
}

NumVector QuaternionComplement::get_gyro_bias() const
{
    return x.segment(4, 3);
}

void QuaternionComplement::get_rpy(double & roll, double & pitch, double & yaw) const
{
    qutils::quat_to_rpy(get_orientation_quaternion(), roll, pitch, yaw);
}

void QuaternionComplement::set_static_accel_gain(double gain)
{
    params.static_accel_gain = gain;
}

void QuaternionComplement::set_static_magn_gain(double gain)
{
    params.static_magn_gain = gain;
}
