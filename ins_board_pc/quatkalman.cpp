#include "quatkalman.h"
#include "wmmwrapper.h"
#include "physconst.h"

#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/assignment.hpp>

#include <QtMath>
#include <QElapsedTimer>

#include <QDebug>

QuaternionKalman::QuaternionKalman(const FilterParams & params)
    : params(params),
      bias_x_ctrl(accum_capacity), bias_y_ctrl(accum_capacity), bias_z_ctrl(accum_capacity)
{
    reset();
    x = NumVector(state_size);
    P = NumMatrix(state_size, state_size);
}

void QuaternionKalman::accumulate(const KalmanInput & z)
{
    bias_x_ctrl.update(z.w[0]);
    bias_y_ctrl.update(z.w[1]);
    bias_z_ctrl.update(z.w[2]);
}

void QuaternionKalman::initialize(const KalmanInput & z)
{
    NumVector qacc = qutils::acceleration_quat(z.a);

    NumMatrix accel_rotator = qutils::quaternion_to_dcm_tr(qacc);
    NumVector l = prod(accel_rotator, z.m);

    NumVector qmag = qutils::magnetometer_quat(l);

    double declination, inclination;
    WrapperWMM::instance().measure(z.geo[0], z.geo[1], z.geo[2], z.day, declination, inclination);

    NumVector qdecl = qutils::declinator_quat(declination);

    NumVector tmp = qutils::quat_multiply(qmag, qdecl);
    NumVector q = qutils::quat_multiply(qacc, tmp);

    // from lb to bl quaternion
    x[0] = q[0];
    x[1] = -q[1];
    x[2] = -q[2];
    x[3] = -q[3];

    x[4] = bias_x_ctrl.get_mean();
    x[5] = bias_y_ctrl.get_mean();
    x[6] = bias_z_ctrl.get_mean();

    x[7] = z.pos[0];
    x[8] = z.pos[1];
    x[9] = z.pos[2];

    x[10] = z.v[0];
    x[11] = z.v[1];
    x[12] = z.v[2];

    x[13] = 0;
    x[14] = 0;
    x[15] = 0;

    P = IdentityMatrix(state_size, state_size) * params.init_params.quat_std * params.init_params.quat_std;

    double bias_var = params.init_params.bias_std * params.init_params.bias_std;
    P(4, 4) = bias_var;
    P(5, 5) = bias_var;
    P(6, 6) = bias_var;

    double position_variance = params.init_params.pos_std * params.init_params.pos_std;
    P(7, 7) = position_variance;
    P(8, 8) = position_variance;
    P(9, 9) = position_variance;

    double velocity_variance = params.init_params.vel_std * params.init_params.vel_std;
    P(10, 10) = velocity_variance;
    P(11, 11) = velocity_variance;
    P(12, 12) = velocity_variance;

    double accel_variance = params.init_params.accel_std * params.init_params.accel_std;
    P(13, 13) = accel_variance;
    P(14, 14) = accel_variance;
    P(15, 15) = accel_variance;
}

void QuaternionKalman::reset()
{
    initialized = false;

    bias_x_ctrl.reset();
    bias_y_ctrl.reset();
    bias_z_ctrl.reset();
}

bool QuaternionKalman::is_initialized()
{
    return initialized;
}

bool QuaternionKalman::bias_estimated()
{
    return bias_x_ctrl.is_saturated() &&
            bias_y_ctrl.is_saturated() &&
            bias_z_ctrl.is_saturated();
}

void QuaternionKalman::step(const KalmanInput & z)
{
    if(initialized)
    {
        update(z);
    }
    else
    {
        accumulate(z);
        if(bias_estimated())
        {
            initialize(z);
            initialized = true;
        }
    }
}

void QuaternionKalman::update(const KalmanInput & z)
{
    NumMatrix F = create_transition_mtx(z);
    NumMatrix Q = create_proc_noise_cov_mtx(z.dt);

    x = prod(F, x);
    normalize_state();

    NumMatrix tmp = prod(F, P);
    P = prod(tmp, trans(F)) + Q;

    NumVector z_pr(measurement_size);

    double lat, lon, alt;
    NumVector predicted_pos = get_position();
    NumVector predicted_orientation = get_orientation_quaternion();

    calculate_geodetic(predicted_pos, lat, lon, alt);

    double mag_magn = norm_2(z.m);

    calculate_accelerometer(predicted_orientation, get_acceleration(), lat, lon, alt, z_pr(0), z_pr(1), z_pr(2));
    calculate_magnetometer(predicted_orientation, lat, lon, alt, z.day, z_pr(3), z_pr(4), z_pr(5));

    z_pr(6) = predicted_pos(0);
    z_pr(7) = predicted_pos(1);
    z_pr(8) = predicted_pos(2);

    NumVector predicted_v = get_velocity();
    calculate_velocity(predicted_v, z_pr(9));

    NumVector z_meas(measurement_size);
    z_meas <<= z.a, z.m / mag_magn, z.pos, norm_2(z.v);

    NumVector y = z_meas - z_pr;

    NumMatrix R = create_meas_noise_cov_mtx(lat, lon, mag_magn);
    NumMatrix H = create_meas_proj_mtx(lat, lon, alt, z.day, predicted_v);

    tmp = prod(H, P);
    NumMatrix S = prod(tmp, trans(H)) + R;

    NumMatrix S_inv(S.size1(), S.size2());

    if(uaux::invert_matrix(S, S_inv))
    {
        tmp = prod(P, trans(H));
        NumMatrix K = prod(tmp, S_inv);

        x += prod(K, y);
        normalize_state();

        tmp = IdentityMatrix(x.size()) - prod(K, H);
        P = prod(tmp, P);
    }
}

void QuaternionKalman::normalize_state()
{
    ublas::project(x, ublas::range(0, 4)) = qutils::quat_normalize(get_orientation_quaternion());
}

NumMatrix QuaternionKalman::create_transition_mtx(const KalmanInput & z)
{
    /* useful constants */
    double dt = z.dt;
    double dt_sq = z.dt * z.dt;
    double dt_2 = z.dt / 2;
    double dt_sq_2 = dt_sq / 2;

    /* constructing state transition matrix */
    NumMatrix V = qutils::skew_symmetric(z.w);

    V *= dt_2;
    V += IdentityMatrix(4);

    NumMatrix K = qutils::quat_delta_mtx(get_orientation_quaternion(), dt_2);

    NumMatrix F(state_size, state_size);

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

    NumMatrix K = qutils::quat_delta_mtx(get_orientation_quaternion(), dt_2);

    NumMatrix Qq = params.proc_params.gyro_std * params.proc_params.gyro_std * prod(K, trans(K));
    NumMatrix Qb = params.proc_params.gyro_bias_std * params.proc_params.gyro_bias_std * IdentityMatrix(3);

    NumMatrix G(9, 3);
    G <<= IdentityMatrix(3) * dt_sq_2,
            IdentityMatrix(3) * dt,
            IdentityMatrix(3);

    NumMatrix Qp = params.proc_params.accel_std * params.proc_params.accel_std * prod(G, trans(G));

    NumMatrix Q(state_size, state_size);

    Q <<= Qq, ZeroMatrix(4, 12),
            ZeroMatrix(3, 4), Qb, ZeroMatrix(3, 9),
            ZeroMatrix(9, 7), Qp;

    return Q;
}

NumMatrix QuaternionKalman::create_meas_noise_cov_mtx(double lat, double lon, double magn_mag)
{
    NumMatrix Ra = params.meas_params.accel_std * params.meas_params.accel_std * IdentityMatrix(3);

    double normalized_magn_std = params.meas_params.magn_std / magn_mag;
    NumMatrix Rm = normalized_magn_std * normalized_magn_std * IdentityMatrix(3);

    double horizontal_linear_std = params.meas_params.gps_cep * 1.2;
    double altitude_std = horizontal_linear_std / 0.53;

    NumMatrix Cel = WrapperWMM::instance().geodetic_to_dcm(lat, lon);
    NumMatrix local_cov = IdentityMatrix(3);
    local_cov(0, 0) = horizontal_linear_std * horizontal_linear_std;
    local_cov(1, 1) = horizontal_linear_std * horizontal_linear_std;
    local_cov(2, 2) = altitude_std * altitude_std;

    NumMatrix tmp = prod(trans(Cel), local_cov);
    NumMatrix Rp = prod(tmp, Cel);

    NumMatrix R(measurement_size, measurement_size);

    R <<= Ra, ZeroMatrix(3, 7),
            ZeroMatrix(3, 3), Rm, ZeroMatrix(3, 4),
            ZeroMatrix(3, 6), Rp, ZeroMatrix(3, 1),
            ZeroMatrix(1, 9), params.meas_params.gps_vel_abs_std * params.meas_params.gps_vel_abs_std;

    return R;
}

NumMatrix QuaternionKalman::create_meas_proj_mtx(double lat, double lon, double alt, QDate day, const NumVector & v)
{
    // 1
    NumMatrix Dac_Dq(3, 4);

    double height_adjust = WrapperWMM::instance().expected_gravity_accel(lat, alt) / phconst::standard_gravity;

    NumVector a = get_acceleration() / phconst::standard_gravity;
    NumVector q = get_orientation_quaternion();

    NumMatrix Cel = WrapperWMM::instance().geodetic_to_dcm(lat, lon);
    NumMatrix Clb = qutils::quaternion_to_dcm_tr(q);
    NumMatrix Ceb = prod(Clb, Cel);

    NumMatrix Ddcm_Dqs = qutils::ddcm_dqs_tr(q);
    NumMatrix Ddcm_Dqx = qutils::ddcm_dqx_tr(q);
    NumMatrix Ddcm_Dqy = qutils::ddcm_dqy_tr(q);
    NumMatrix Ddcm_Dqz = qutils::ddcm_dqz_tr(q);

    NumVector tmp = prod(Cel, a);

    NumVector col_s = column(Ddcm_Dqs, 2) * height_adjust + prod(Ddcm_Dqs, tmp);
    NumVector col_x = column(Ddcm_Dqx, 2) * height_adjust + prod(Ddcm_Dqx, tmp);
    NumVector col_y = column(Ddcm_Dqy, 2) * height_adjust + prod(Ddcm_Dqy, tmp);
    NumVector col_z = column(Ddcm_Dqz, 2) * height_adjust + prod(Ddcm_Dqz, tmp);

    Dac_Dq <<= ublas::traverse_policy::by_column(), col_s, col_x, col_y, col_z;

    // 2
    NumMatrix Dac_Dpos(3, 3);

    NumMatrix Dgeo_Dpos = WrapperWMM::instance().dgeo_dpos(lat, lon, alt);
    NumMatrix Ddcm_Dlat = WrapperWMM::instance().dcm_lat_partial(lat, lon);
    NumMatrix Ddcm_Dlon = WrapperWMM::instance().dcm_lon_partial(lat, lon);

    NumMatrix Dcel_Dx = Dgeo_Dpos(0, 0) * Ddcm_Dlat + Dgeo_Dpos(1, 0) * Ddcm_Dlon;
    NumMatrix Dcel_Dy = Dgeo_Dpos(0, 1) * Ddcm_Dlat + Dgeo_Dpos(1, 1) * Ddcm_Dlon;
    NumMatrix Dcel_Dz = Dgeo_Dpos(0, 2) * Ddcm_Dlat + Dgeo_Dpos(1, 2) * Ddcm_Dlon;

    tmp = prod(Dcel_Dx, a);
    col_x = prod(Clb, tmp);
    tmp = prod(Dcel_Dy, a);
    col_y = prod(Clb, tmp);
    tmp = prod(Dcel_Dz, a);
    col_z = prod(Clb, tmp);

    Dac_Dpos <<= ublas::traverse_policy::by_column(), col_x, col_y, col_z;

    // 3
    NumMatrix Dac_Da = Ceb / phconst::standard_gravity;

    // 4
    NumMatrix Dm_Dq(3, 4);

    NumVector earth_mag = WrapperWMM::instance().expected_mag(lat, lon, alt, day);

    col_s = prod(Ddcm_Dqs, earth_mag);
    col_x = prod(Ddcm_Dqx, earth_mag);
    col_y = prod(Ddcm_Dqy, earth_mag);
    col_z = prod(Ddcm_Dqz, earth_mag);

    Dm_Dq <<= ublas::traverse_policy::by_column(), col_s, col_x, col_y, col_z;

    // 5
    NumMatrix Dpos_Dpos = IdentityMatrix(3);

    // 6
    NumMatrix Dv_Dv(1, 3);
    double v_abs = norm_2(v);

    if(v_abs > 0)
    {
        Dv_Dv <<= v[0] / v_abs, v[1] / v_abs, v[2] / v_abs;
    }
    else
    {
        Dv_Dv <<= 0, 0, 0;
    }

    // Combine
    NumMatrix H(measurement_size, state_size);
    H <<= Dac_Dq, ZeroMatrix(3, 3), Dac_Dpos, ZeroMatrix(3, 3), Dac_Da,
            Dm_Dq, ZeroMatrix(3, 12),
            ZeroMatrix(3, 7), Dpos_Dpos, ZeroMatrix(3, 6),
            ZeroMatrix(1, 10), Dv_Dv, ZeroMatrix(1, 3);

    return H;
}

void QuaternionKalman::calculate_geodetic(const NumVector & position,
                                          double & lat, double & lon, double & alt)
{
    WrapperWMM::instance().cartesian_to_geodetic(position, lat, lon, alt);
}

void QuaternionKalman::calculate_accelerometer(const NumVector & orientation_quat, const NumVector & acceleration,
                             double lat, double lon, double alt,
                             double & ax, double & ay, double & az)
{
    double height_adjust = WrapperWMM::instance().expected_gravity_accel(lat, alt) / phconst::standard_gravity;

    NumMatrix Clb = qutils::quaternion_to_dcm_tr(orientation_quat);
    NumMatrix Cel = WrapperWMM::instance().geodetic_to_dcm(lat, lon);
    NumMatrix tmp = prod(Clb, Cel);
    NumVector movement_component = prod(tmp, acceleration / phconst::standard_gravity);
    NumVector g(3);
    g <<= 0, 0, height_adjust;

    NumVector gravity_component = prod(Clb, g);

    NumVector rot_accel = gravity_component + movement_component;

    ax = rot_accel(0);
    ay = rot_accel(1);
    az = rot_accel(2);
}

void QuaternionKalman::calculate_magnetometer(const NumVector & orientation_quat,
                                              double lat, double lon, double alt, QDate day,
                                              double & mx, double & my, double & mz)
{
    NumVector rot_magn = prod(qutils::quaternion_to_dcm_tr(orientation_quat),
                                          WrapperWMM::instance().expected_mag(lat, lon, alt, day));

    mx = rot_magn(0);
    my = rot_magn(1);
    mz = rot_magn(2);
}

void QuaternionKalman::calculate_velocity(const NumVector & velocity, double & vel)
{
    vel = norm_2(velocity);
}

NumVector QuaternionKalman::get_state()
{
    return x;
}

NumVector QuaternionKalman::get_orientation_quaternion()
{
    return ublas::project(x, ublas::range(0, 4));
}

NumVector QuaternionKalman::get_gyro_bias()
{
    return ublas::project(x, ublas::range(4, 7));
}

NumVector QuaternionKalman::get_position()
{
    return ublas::project(x, ublas::range(7, 10));
}

NumVector QuaternionKalman::get_velocity()
{
    return ublas::project(x, ublas::range(10, 13));
}

NumVector QuaternionKalman::get_acceleration()
{
    return ublas::project(x, ublas::range(13, 16));
}

void QuaternionKalman::get_rpy(double & roll, double & pitch, double & yaw)
{
    qutils::quat_to_rpy(get_orientation_quaternion(), roll, pitch, yaw);
}

void QuaternionKalman::get_geodetic(double & lat, double & lon, double & alt)
{
    calculate_geodetic(get_position(), lat, lon, alt);
}

void QuaternionKalman::set_proc_gyro_std(double std)
{
    params.proc_params.gyro_std = std;
}

void QuaternionKalman::set_proc_gyro_bias_std(double std)
{
    params.proc_params.gyro_bias_std = std;
}

void QuaternionKalman::set_proc_accel_std(double std)
{
    params.proc_params.accel_std = std;
}

void QuaternionKalman::set_meas_accel_std(double std)
{
    params.meas_params.accel_std = std;
}

void QuaternionKalman::set_meas_magn_std(double std)
{
    params.meas_params.magn_std = std;
}

void QuaternionKalman::set_meas_pos_std(double std)
{
    params.meas_params.gps_cep = std;
}

void QuaternionKalman::set_meas_vel_std(double std)
{
    params.meas_params.gps_vel_abs_std = std;
}

void QuaternionKalman::set_init_quat_std(double std)
{
    params.init_params.quat_std = std;
}

void QuaternionKalman::set_init_bias_std(double std)
{
    params.init_params.bias_std = std;
}

void QuaternionKalman::set_init_pos_std(double std)
{
    params.init_params.pos_std = std;
}

void QuaternionKalman::set_init_vel_std(double std)
{
    params.init_params.vel_std = std;
}

void QuaternionKalman::set_init_accel_std(double std)
{
    params.init_params.accel_std = std;
}
