#include "quatkalman.h"

#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/assignment.hpp>

#include <iostream>
#include <boost/numeric/ublas/io.hpp>

#include <QtMath>

QuaternionKalman::QuaternionKalman(double proc_gyro_std, double proc_gyro_bias_std,
                                   double proc_accel_std, double meas_accel_std,
                                   double meas_magn_std, double meas_cep,
                                   double meas_vel_std)
    : proc_gyro_std(proc_gyro_std), proc_gyro_bias_std(proc_gyro_bias_std),
      proc_accel_std(proc_accel_std), meas_accel_std(meas_accel_std),
      meas_magn_std(meas_magn_std), meas_cep(meas_cep), meas_vel_std(meas_vel_std)
{
    reset();
}

void QuaternionKalman::initialize(const KalmanInput & z1)
{

}

void QuaternionKalman::reset()
{
    initialized = false;
    first_sample_received = false;
}

void QuaternionKalman::step(const KalmanInput & z)
{
    if(initialized)
    {
        update(z);
    }
    else if(first_sample_received)
    {
        initialize(z);
        initialized = true;
    }
    else
    {
        z0 = z;
        first_sample_received = true;
    }
}

void QuaternionKalman::update(const KalmanInput & z)
{
    NumMatrix F = create_transition_mtx(z);
    NumMatrix Q = create_proc_noise_cov_mtx(z);
    NumMatrix R = create_meas_noise_cov_mtx(z);

    x = prod(F, x);

    NumVector z_pr(10);

    calculate_geodetic(get_position(), z_pr(6), z_pr(7), z_pr(8));
    calculate_accelerometer(get_orinetation_quaternion(), get_acceleration(), z_pr(6), z_pr(7), z_pr(0), z_pr(1), z_pr(2));
    calculate_magnetometer(get_orinetation_quaternion(), z_pr(3), z_pr(4), z_pr(5));
    calculate_velocity(get_velocity(), z_pr(9));

    NumMatrix H = create_meas_proj_mtx(z, z_pr);
}

NumMatrix QuaternionKalman::create_quat_bias_mtx(double dt_2)
{
    double q_s = x[0];
    double q_x = x[1];
    double q_y = x[2];
    double q_z = x[3];

    NumMatrix K(4, 3);
    K <<= q_x,  q_y,  q_z,
           -q_s,  q_z, -q_y,
           -q_z, -q_s,  q_x,
            q_y, -q_x, -q_s;

    K *= dt_2;

    return K;
}

NumMatrix QuaternionKalman::create_transition_mtx(const KalmanInput & z)
{
    /* useful constants */
    double dt = z.dt;
    double dt_sq = z.dt * z.dt;
    double dt_2 = z.dt / 2;
    double dt_sq_2 = dt_sq / 2;

    /* constructing state transition matrix */
    NumMatrix V(4, 4);
    V <<=    0,      -z.w[0], -z.w[1],  z.w[2],
             z.w[0],  0,       z.w[2], -z.w[1],
             z.w[1], -z.w[2],  0,       z.w[0],
             z.w[2],  z.w[1], -z.w[0],  0;

    V *= dt_2;
    V += IdentityMatrix(4);

    NumMatrix K = create_quat_bias_mtx(dt_2);

    NumMatrix F(16, 16);

    F <<= V, K, ZeroMatrix(4, 9),
            ZeroMatrix(3, 4), IdentityMatrix(3), ZeroMatrix(3, 9),
            ZeroMatrix(3, 7), IdentityMatrix(3), dt * IdentityMatrix(3), dt_sq_2 * IdentityMatrix(3),
            ZeroMatrix(3, 10), IdentityMatrix(3), dt * IdentityMatrix(3),
            ZeroMatrix(3, 13), IdentityMatrix(3);

    return F;
}

NumMatrix QuaternionKalman::create_proc_noise_cov_mtx(const KalmanInput & z)
{
    /* useful constants */
    double dt = z.dt;
    double dt_sq = z.dt * z.dt;
    double dt_2 = z.dt / 2;
    double dt_sq_2 = dt_sq / 2;

    NumMatrix K = create_quat_bias_mtx(dt_2);

    NumMatrix Qq = proc_gyro_std * proc_gyro_std * prod(K, trans(K));
    NumMatrix Qb = proc_gyro_bias_std * proc_gyro_bias_std * IdentityMatrix(3);

    NumMatrix G(9, 1);
    G <<= dt_sq_2, dt_sq_2, dt_sq_2,
            dt, dt, dt,
            1, 1, 1;

    NumMatrix Qp = proc_accel_std * proc_accel_std * prod(G, trans(G));

    NumMatrix Q(16, 16);

    Q <<= Qq, ZeroMatrix(4, 12),
            ZeroMatrix(3, 4), Qb, ZeroMatrix(3, 9),
            ZeroMatrix(9, 7), Qp;

    return Q;
}

NumMatrix QuaternionKalman::create_meas_noise_cov_mtx(const KalmanInput & z)
{
    NumMatrix Ra = meas_accel_std * meas_accel_std * IdentityMatrix(3);
    NumMatrix Rm = meas_magn_std * meas_magn_std * IdentityMatrix(3);

    double horizontal_linear_std = meas_cep * 1.2;
    double altitude_std = horizontal_linear_std / 0.53;

    double latitude_std = horizontal_linear_std / earth_rad;
    double longitude_std = qAcos(qCos(latitude_std) / qPow(qCos(z.lat), 2) - qPow(qTan(z.lat), 2));

    NumMatrix R = IdentityMatrix(10);

    R <<= Ra, ZeroMatrix(3, 7),
            ZeroMatrix(3, 3), Rm, ZeroMatrix(3, 4),
            ZeroMatrix(4, 10);

    R(6, 6) = latitude_std * latitude_std;
    R(7, 7) = longitude_std * longitude_std;
    R(8, 8) = altitude_std * altitude_std;
    R(9, 9) = meas_vel_std * meas_vel_std;

    return R;
}

NumMatrix QuaternionKalman::create_meas_proj_mtx(const KalmanInput & z, const NumVector & predicted_z)
{

}

void QuaternionKalman::calculate_accelerometer(const NumVector & orientation_quat, const NumVector & acceleration,
                             double lat, double lon,
                             double & ax, double & ay, double & az)
{

}

void QuaternionKalman::calculate_magnetometer(const NumVector & orientation_quat,
                            double & mx, double & my, double & mz)
{

}

void QuaternionKalman::calculate_geodetic(const NumVector & position,
                        double & lat, double & lon, double & alt)
{

}

void QuaternionKalman::calculate_velocity(const NumVector & velocity, double & vel)
{

}

NumVector QuaternionKalman::get_state()
{
    return x;
}

NumVector QuaternionKalman::get_orinetation_quaternion()
{
    return ublas::vector_range<NumVector>(x, ublas::range(0, 4));
}

NumVector QuaternionKalman::get_gyro_bias()
{
    return ublas::vector_range<NumVector>(x, ublas::range(4, 7));
}

NumVector QuaternionKalman::get_position()
{
    return ublas::vector_range<NumVector>(x, ublas::range(7, 10));
}

NumVector QuaternionKalman::get_velocity()
{
    return ublas::vector_range<NumVector>(x, ublas::range(10, 13));
}

NumVector QuaternionKalman::get_acceleration()
{
    return ublas::vector_range<NumVector>(x, ublas::range(13, 16));
}
