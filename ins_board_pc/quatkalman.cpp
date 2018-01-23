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
    x = NumVector(16);
    P = NumMatrix(16, 16);
}

void QuaternionKalman::initialize(const KalmanInput & z1)
{
    double ax = z1.a[0];
    double ay = z1.a[1];
    double az = z1.a[2];

    double mx = z1.m[0];
    double my = z1.m[1];
    double mz = z1.m[2];

    NumVector qacc(4);
    if(az >= 0)
    {
        qacc[0] = qSqrt( (az + 1) / 2);
        qacc[1] = - ay / qSqrt(2 * (az + 1));
        qacc[2] = ax / qSqrt(2 * (az + 1));
        qacc[3] = 0;
    }
    else
    {
        qacc[0] = - ay / qSqrt(2 * (1 - az));
        qacc[1] = qSqrt( (1 - az) / 2);
        qacc[2] = 0;
        qacc[3] = ax / qSqrt(2 * (1 - az));
    }

    NumVector qmag(4);
    //double G =

    /*
    double lat, lon, alt;
    NumVector pos(3);
    pos <<= z1.pos[0], z1.pos[1], z1.pos[2];
    calculate_geodetic(pos, lat, lon, alt);
    NumVector orientation(4);
    orientation <<= 1, 0, 0, 0;
    double mx, my, mz;
    calculate_magnetometer(orientation, norm_2(z1.m), lat, lon, alt, z1.day, mx, my, mz);
    double ax, ay, az;
    NumVector acceleration(3);
    acceleration <<= 0, 0, 0;
    calculate_accelerometer(orientation, acceleration, lat, lon, ax, ay, az);
    int g = 0;
    */
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
    NumMatrix Q = create_proc_noise_cov_mtx(z.dt);

    x = prod(F, x);

    NumVector z_pr(10);

    double lat, lon, alt;
    NumVector predicted_pos = get_position();
    NumVector predicted_orientation = get_orinetation_quaternion();

    calculate_geodetic(predicted_pos, lat, lon, alt);

    calculate_accelerometer(predicted_orientation, get_acceleration(), lat, lon, z_pr(0), z_pr(1), z_pr(2));
    calculate_magnetometer(predicted_orientation, norm_2(z.m), lat, lon, alt, z.day, z_pr(3), z_pr(4), z_pr(5));

    z_pr(6) = predicted_pos(0);
    z_pr(7) = predicted_pos(1);
    z_pr(8) = predicted_pos(2);

    calculate_velocity(get_velocity(), z_pr(9));

    NumMatrix R = create_meas_noise_cov_mtx(lat, lon);
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

NumMatrix QuaternionKalman::create_proc_noise_cov_mtx(double dt)
{
    /* useful constants */
    double dt_sq = dt * dt;
    double dt_2 = dt / 2;
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

NumMatrix QuaternionKalman::create_meas_noise_cov_mtx(double lat, double lon)
{
    NumMatrix Ra = meas_accel_std * meas_accel_std * IdentityMatrix(3);
    NumMatrix Rm = meas_magn_std * meas_magn_std * IdentityMatrix(3);

    double horizontal_linear_std = meas_cep * 1.2;
    double altitude_std = horizontal_linear_std / 0.53;

    NumMatrix Cel = geodetic_to_dcm(lat, lon);
    NumMatrix local_cov = IdentityMatrix(3);
    local_cov(0, 0) = horizontal_linear_std * horizontal_linear_std;
    local_cov(1, 1) = horizontal_linear_std * horizontal_linear_std;
    local_cov(2, 2) = altitude_std * altitude_std;

    NumMatrix tmp = prod(trans(Cel), local_cov);
    NumMatrix Rp = prod(tmp, Cel);

    NumMatrix R = IdentityMatrix(10);

    R <<= Ra, ZeroMatrix(3, 7),
            ZeroMatrix(3, 3), Rm, ZeroMatrix(3, 4),
            ZeroMatrix(3, 6), Rp, ZeroMatrix(3, 1),
            ZeroMatrix(1, 9), meas_vel_std * meas_vel_std;

    return R;
}

NumMatrix QuaternionKalman::create_meas_proj_mtx(const KalmanInput & z, const NumVector & predicted_z)
{

}

void QuaternionKalman::calculate_geodetic(const NumVector & position,
                                          double & lat, double & lon, double & alt)
{
    wmm.cartesian_to_geodetic(position[0], position[1], position[2], lat, lon, alt);
}

void QuaternionKalman::calculate_accelerometer(const NumVector & orientation_quat, const NumVector & acceleration,
                             double lat, double lon,
                             double & ax, double & ay, double & az)
{
    NumMatrix Clb = quaternion_to_dcm(orientation_quat);
    NumMatrix Cel = geodetic_to_dcm(lat, lon);
    NumMatrix tmp = prod(Clb, Cel);
    NumVector movement_component = prod(tmp, acceleration);
    NumVector g(3);
    g <<= 0, 0, 1;
    NumVector gravity_component = prod(Clb, g);

    NumVector rot_accel = gravity_component + movement_component;

    ax = rot_accel(0);
    ay = rot_accel(1);
    az = rot_accel(2);
}

void QuaternionKalman::calculate_magnetometer(const NumVector & orientation_quat, double magnitude,
                                              double lat, double lon, double alt, QDate day,
                                              double & mx, double & my, double & mz)
{
    NumVector rot_magn = magnitude * prod(quaternion_to_dcm(orientation_quat),
                                          expected_mag(lat, lon, alt, day));

    mx = rot_magn(0);
    my = rot_magn(1);
    mz = rot_magn(2);
}

void QuaternionKalman::calculate_velocity(const NumVector & velocity, double & vel)
{
    vel = norm_2(velocity);
}

NumMatrix QuaternionKalman::quaternion_to_dcm(const NumVector & quaternion)
{
    double qs = quaternion[0];
    double qx = quaternion[1];
    double qy = quaternion[2];
    double qz = quaternion[3];

    NumMatrix DCM(3, 3);

    double qss = qs * qs;
    double qxx = qx * qx;
    double qyy = qy * qy;
    double qzz = qz * qz;
    double qsx = qs * qx;
    double qsy = qs * qy;
    double qsz = qs * qz;
    double qxy = qx * qy;
    double qxz = qx * qz;
    double qyz = qy * qz;

    DCM(0, 0) = qss + qxx - qyy - qzz;
    DCM(0, 1) = 2 * (qxy - qsz);
    DCM(0, 2) = 2 * (qxz + qsy);
    DCM(1, 0) = 2 * (qxy + qsz);
    DCM(1, 1) = qss - qxx + qyy - qzz;
    DCM(1, 2) = 2 * (qyz - qsx);
    DCM(2, 0) = 2 * (qxz - qsy);
    DCM(2, 1) = 2 * (qyz + qsx);
    DCM(2, 2) = qss - qxx - qyy + qzz;

    return DCM;
}

NumMatrix QuaternionKalman::geodetic_to_dcm(double lat, double lon)
{
    double clat = qCos(lat);
    double slat = qSin(lat);
    double clon = qCos(lon);
    double slon = qSin(lon);

    NumMatrix DCM(3, 3);

    DCM <<= -slon, clon, 0,
            -clon * slat, -slon * slat, clat,
            clon * clat, slon * clat, slat;

    return DCM;
}

NumVector QuaternionKalman::expected_mag(double lat, double lon, double alt, QDate day)
{
    double declination, inclination;
    wmm.measure(lat, lon, alt, day, declination, inclination);

    double sdecl = qSin(declination);
    double cdecl = qCos(declination);
    double sincl = qSin(inclination);
    double cincl = qCos(inclination);

    NumVector RES(3);
    RES <<= sdecl * cincl, cdecl * cincl, -sincl;
    return RES;
}

NumMatrix QuaternionKalman::ddcm_dqs(const NumVector & quaternion)
{
    double qs = quaternion[0];
    double qx = quaternion[1];
    double qy = quaternion[2];
    double qz = quaternion[3];

    NumMatrix RES(3, 3);

    RES <<= qs, -qz, qy,
            qz, qs, -qx,
            -qy, qx, qs;

    RES *= 2;

    return RES;
}

NumMatrix QuaternionKalman::ddcm_dqx(const NumVector & quaternion)
{
    double qs = quaternion[0];
    double qx = quaternion[1];
    double qy = quaternion[2];
    double qz = quaternion[3];

    NumMatrix RES(3, 3);

    RES <<= qx, qy, qz,
            qy, -qx, -qs,
            qz, qs, -qx;

    RES *= 2;

    return RES;
}

NumMatrix QuaternionKalman::ddcm_dqy(const NumVector & quaternion)
{
    double qs = quaternion[0];
    double qx = quaternion[1];
    double qy = quaternion[2];
    double qz = quaternion[3];

    NumMatrix RES(3, 3);

    RES <<= -qy, qx, qs,
            qx, qy, qz,
            -qs, qz, -qy;

    RES *= 2;

    return RES;
}

NumMatrix QuaternionKalman::ddcm_dqz(const NumVector & quaternion)
{
    double qs = quaternion[0];
    double qx = quaternion[1];
    double qy = quaternion[2];
    double qz = quaternion[3];

    NumMatrix RES(3, 3);

    RES <<= -qz, -qs, qx,
            qs, -qz, qy,
            qx, qy, qz;

    RES *= 2;

    return RES;
}

NumMatrix QuaternionKalman::dcm_lat_partial(double lat, double lon)
{
    double clat = qCos(lat);
    double slat = qSin(lat);
    double clon = qCos(lon);
    double slon = qSin(lon);

    NumMatrix RES(3, 3);

    RES <<= 0, 0, 0,
            -clon * clat, -slon * clat, -slat,
            -clon * slat, -slon * slat, clat;

    return RES;
}

NumMatrix QuaternionKalman::dcm_lon_partial(double lat, double lon)
{
    double clat = qCos(lat);
    double slat = qSin(lat);
    double clon = qCos(lon);
    double slon = qSin(lon);

    NumMatrix RES(3, 3);

    RES <<= -clon, -slon, 0,
            slon * slat, -clon * slat, 0,
            -slon * clat, clon * clat, 0;

    return RES;
}

NumMatrix QuaternionKalman::dgeo_dpos(double lat, double lon, double alt)
{
    double clat = qCos(lat);
    double slat = qSin(lat);
    double clon = qCos(lon);
    double slon = qSin(lon);

    double bracket = qPow(1 - wmm.ellip_epssq() * slat * slat, 1.5);
    double norm_rad = wmm.ellip_a() / qSqrt(1 - wmm.ellip_epssq() * slat * slat);
    double common_mult = bracket / (wmm.ellip_a() * (wmm.ellip_epssq() - 1) - alt * bracket);
    double common_mult_2 = 1 / (norm_rad + alt);

    NumMatrix RES(3, 3);

    RES(0, 0) = common_mult / (slat * clon);
    RES(0, 1) = common_mult / (slat * slon);
    RES(0, 2) = -common_mult / clat;

    RES(1, 0) = -common_mult_2 / (clat * slon);
    RES(1, 1) = common_mult_2 / (clat * clon);
    RES(1, 2) = 0;

    RES(2, 0) = 1 / (clat * clon);
    RES(2, 1) = 1 / (clat * slon);
    RES(2, 2) = 1 / slat;

    return RES;
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
