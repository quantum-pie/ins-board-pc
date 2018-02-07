#include "quatcomplement.h"

#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/assignment.hpp>

#include <QtMath>

QuatComplement::QuatComplement(const FilterParams & params)
    : params(params),
      bias_x_ctrl(accum_capacity), bias_y_ctrl(accum_capacity), bias_z_ctrl(accum_capacity)
{
    reset();
    x = NumVector(state_size);
}

void QuatComplement::accumulate(const ComplementInput & z)
{
    bias_x_ctrl.update(z.w[0]);
    bias_y_ctrl.update(z.w[1]);
    bias_z_ctrl.update(z.w[2]);
}

bool QuatComplement::bias_estimated()
{
    return bias_x_ctrl.is_saturated() &&
            bias_y_ctrl.is_saturated() &&
            bias_z_ctrl.is_saturated();
}

void QuatComplement::initialize(const ComplementInput & z)
{
    NumVector qacc = qutils::acceleration_quat(z.a);

    NumMatrix accel_rotator = qutils::quaternion_to_dcm_tr(qacc);
    NumVector l = prod(accel_rotator, z.m);

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

void QuatComplement::reset()
{
    initialized = false;

    bias_x_ctrl.reset();
    bias_y_ctrl.reset();
    bias_z_ctrl.reset();
}

bool QuatComplement::is_initialized()
{
    return initialized;
}

void QuatComplement::step(const ComplementInput & z)
{
    accumulate(z);
    if(initialized)
    {
        update(z);
    }
    else if(bias_estimated())
    {
        initialize(z);
        initialized = true;
    }
}

void QuatComplement::update(const ComplementInput & z)
{
    /* useful constants */
    double dt_2 = z.dt / 2;

    /* constructing the quaternion propagation matrix */
    NumMatrix V = qutils::skew_symmetric(z.w);

    V *= dt_2;
    V += IdentityMatrix(4);

    NumMatrix K = qutils::quat_delta_mtx(get_orientation_quaternion(), dt_2);

    NumMatrix F(state_size, state_size);

    F <<= V, K,
          ZeroMatrix(3, 4), IdentityMatrix(3);

    /* propagate quaternion */
    x = prod(F, x);

    /* update biases */
    x[4] = bias_x_ctrl.get_mean();
    x[5] = bias_y_ctrl.get_mean();
    x[6] = bias_z_ctrl.get_mean();

    normalize_state();

    NumVector ident_quat = qutils::identity_quaternion();

    /* extract predicted quaternion */
    NumVector q_pred = get_orientation_quaternion();

    NumVector a_norm = z.a / norm_2(z.a);

    /* predict gravity */
    NumVector g_pred = prod(qutils::quaternion_to_dcm(q_pred), a_norm);
    NumVector qacc_delta = qutils::quat_conjugate(qutils::acceleration_quat(g_pred));
    NumVector qacc_corr = qutils::lerp(ident_quat, qacc_delta, calculate_gain(z.a));
    NumVector q_corr = qutils::quat_multiply(qacc_corr, q_pred);

    /* predict magnetic vector */
    NumVector mag_pred = prod(qutils::quaternion_to_dcm(q_corr), z.m);
    NumVector qmag_delta = qutils::quat_conjugate(qutils::magnetometer_quat(mag_pred));
    NumVector qmag_corr = qutils::lerp(ident_quat, qmag_delta, params.static_magn_gain);
    q_corr = qutils::quat_multiply(qmag_corr, q_corr);

    /* finish */
    x[0] = q_corr[0];
    x[1] = q_corr[1];
    x[2] = q_corr[2];
    x[3] = q_corr[3];
}

void QuatComplement::normalize_state()
{
    ublas::project(x, ublas::range(0, 4)) = qutils::quat_normalize(get_orientation_quaternion());
}

double QuatComplement::calculate_gain(const NumVector & accel)
{
    // TODO
    return params.static_accel_gain;
}

NumVector QuatComplement::get_state()
{
    return x;
}

NumVector QuatComplement::get_orientation_quaternion()
{
    return ublas::project(x, ublas::range(0, 4));
}

NumVector QuatComplement::get_gyro_bias()
{
    return ublas::project(x, ublas::range(4, 7));
}

void QuatComplement::get_rpy(double & roll, double & pitch, double & yaw)
{
    qutils::quat_to_rpy(get_orientation_quaternion(), roll, pitch, yaw);
}
