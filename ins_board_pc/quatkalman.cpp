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
    initialize_wmm();

    NumVector magn = expected_mag(qDegreesToRadians(59.9013625), qDegreesToRadians(30.2825023), 0e-3);
}

void QuaternionKalman::initialize_wmm()
{
    magnetic_models = new MAGtype_MagneticModel*[1];

    MAG_robustReadMagModels(const_cast<char*>("res/WMM.COF"),
                            &magnetic_models, 1);

    int n_max = 0;
    if(n_max < magnetic_models[0]->nMax)
        n_max = magnetic_models[0]->nMax;

    int terms = ((n_max + 1) * (n_max + 2) / 2);

    timed_magnetic_model = MAG_AllocateModelMemory(terms);

    MAG_SetDefaults(&ellip, &geoid);
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
    calculate_magnetometer(get_orinetation_quaternion(), norm_2(z.m), z_pr(3), z_pr(4), z_pr(5));
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

    double latitude_std = horizontal_linear_std / (ellip.re * 1e3);
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

void QuaternionKalman::calculate_magnetometer(const NumVector & orientation_quat, double magnitude,
                            double & mx, double & my, double & mz)
{
    NumVector rot_magn = magnitude * prod(quaternion_to_dcm(orientation_quat),
                                          expected_mag(qDegreesToRadians(59.9013625), qDegreesToRadians(30.2825023), 0e-3));

    mx = rot_magn(0);
    my = rot_magn(1);
    mx = rot_magn(2);
}

void QuaternionKalman::calculate_geodetic(const NumVector & position,
                        double & lat, double & lon, double & alt)
{

}

void QuaternionKalman::calculate_velocity(const NumVector & velocity, double & vel)
{

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

NumVector QuaternionKalman::expected_mag(double lat, double lon, double alt)
{
    MAGtype_CoordGeodetic geodetic_coord;
    geodetic_coord.UseGeoid = 0;
    geodetic_coord.phi = qRadiansToDegrees(lat);
    geodetic_coord.lambda = qRadiansToDegrees(lon);
    geodetic_coord.HeightAboveEllipsoid = alt * 1e-3;

    MAGtype_CoordSpherical spherical_coord;
    MAGtype_GeoMagneticElements result;

    MAG_GeodeticToSpherical(ellip, geodetic_coord, &spherical_coord);

    MAGtype_Date date;
    date.Year = 2018;
    date.Month = 1;
    date.Day = 17;

    char err_msg[255];
    MAG_DateToYear(&date, err_msg);
    MAG_TimelyModifyMagneticModel(date, magnetic_models[0], timed_magnetic_model);
    MAG_Geomag(ellip, spherical_coord, geodetic_coord, timed_magnetic_model, &result);

    double declination = qDegreesToRadians(result.Decl);
    double inclination = qDegreesToRadians(result.Incl);

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

    double bracket = qPow(1 - ellip.epssq * slat * slat, 1.5);
    double norm_rad = ellip.a * 1e3 / qSqrt(1 - ellip.epssq * slat * slat);
    double common_mult = bracket / (ellip.a * 1e3 * (ellip.epssq - 1) - alt * bracket);
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
