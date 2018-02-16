#include "quatkalman.h"
#include "wmmwrapper.h"
#include "physconst.h"
#include "quaternions.h"

#include <Eigen/Dense>

#include <QtMath>
#include <QDebug>

const int QuaternionKalman::state_size = 16;
const int QuaternionKalman::measurement_size = 10;

QuaternionKalman::QuaternionKalman(const FilterParams & par)
    : AbstractKalmanOrientationFilter(par.accum_capacity),
      AbstractKalmanPositionFilter(),
      params(par)
{
    x = NumVector(state_size);
    P = NumMatrix(state_size, state_size);
}

QuaternionKalman::~QuaternionKalman()
{

}

void QuaternionKalman::accumulate(const FilterInput & z)
{
    AbstractKalmanOrientationFilter::accumulate(z);
    AbstractKalmanPositionFilter::accumulate(z);
}

void QuaternionKalman::reset()
{
    AbstractKalmanOrientationFilter::reset();
    AbstractKalmanPositionFilter::reset();
}

void QuaternionKalman::initialize(const FilterInput & z)
{
    AbstractKalmanOrientationFilter::initialize(z);
    AbstractKalmanPositionFilter::initialize(z);

    NumVector qacc = qutils::acceleration_quat(z.a);

    NumMatrix accel_rotator = qutils::quaternion_to_dcm_tr(qacc);
    NumVector l = accel_rotator * z.m;

    NumVector qmag = qutils::magnetometer_quat(l);

    double declination, inclination, magn;
    WrapperWMM::instance().measure(z.geo[0], z.geo[1], z.geo[2], z.day, declination, inclination, magn);

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

    P = NumMatrix::Zero(state_size, state_size);
    P(0, 0) = params.init_params.qs_std * params.init_params.qs_std;
    P(1, 1) = params.init_params.qx_std * params.init_params.qx_std;
    P(2, 2) = params.init_params.qy_std * params.init_params.qy_std;
    P(3, 3) = params.init_params.qz_std * params.init_params.qz_std;

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

void QuaternionKalman::step(const FilterInput & z)
{
    if(is_initialized())
    {
        update(z);
    }
    else
    {
        accumulate(z);
        if(bias_estimated())
        {
            initialize(z);
        }
    }
}

void QuaternionKalman::update(const FilterInput & z)
{
    NumMatrix F = create_transition_mtx(z);
    NumMatrix Q = create_proc_noise_cov_mtx(z.dt);

    x = F * x;
    normalize_state();

    P = F * P * F.transpose() + Q;

    NumVector z_pr(measurement_size);

    double lat, lon, alt;
    NumVector predicted_pos = get_position();
    NumVector predicted_orientation = get_orientation_quaternion();

    calculate_geodetic(predicted_pos, lat, lon, alt);

    calculate_accelerometer(predicted_orientation, get_acceleration(), lat, lon, alt, z_pr(0), z_pr(1), z_pr(2));
    calculate_magnetometer(predicted_orientation, lat, lon, alt, z.day, z_pr(3), z_pr(4), z_pr(5));

    z_pr(6) = predicted_pos(0);
    z_pr(7) = predicted_pos(1);
    z_pr(8) = predicted_pos(2);

    NumVector predicted_v = get_velocity();
    calculate_velocity(predicted_v, z_pr(9));

    NumVector z_meas(measurement_size);
    z_meas << z.a, z.m, z.pos, z.v.norm();

    NumVector y = z_meas - z_pr;

    NumMatrix R = create_meas_noise_cov_mtx(lat, lon, alt, z.day);
    NumMatrix H = create_meas_proj_mtx(lat, lon, alt, z.day, predicted_v);

    NumMatrix S = H * P * H.transpose() + R;
    NumMatrix K = P * H.transpose() * S.inverse();

    x += K * y;
    normalize_state();

    NumMatrix tmp = NumMatrix::Identity(x.size(), x.size()) - K * H;
    P = tmp * P * tmp.transpose() + K * R * K.transpose();
}

void QuaternionKalman::normalize_state()
{
    x.segment(0, 4) = qutils::quat_normalize(get_orientation_quaternion());
}

NumMatrix QuaternionKalman::create_transition_mtx(const FilterInput & z) const
{
    /* useful constants */
    double dt = z.dt;
    double dt_sq = z.dt * z.dt;
    double dt_2 = z.dt / 2;
    double dt_sq_2 = dt_sq / 2;

    /* constructing state transition matrix */
    NumMatrix V = qutils::skew_symmetric(z.w);

    V *= dt_2;
    V += NumMatrix::Identity(4, 4);

    NumMatrix K = qutils::quat_delta_mtx(get_orientation_quaternion(), dt_2);

    NumMatrix F(state_size, state_size);

    F << V, K, NumMatrix::Zero(4, 9),
            NumMatrix::Zero(3, 4), NumMatrix::Identity(3, 3), NumMatrix::Zero(3, 9),
            NumMatrix::Zero(3, 7), NumMatrix::Identity(3, 3), dt * NumMatrix::Identity(3, 3), dt_sq_2 * NumMatrix::Identity(3, 3),
            NumMatrix::Zero(3, 10), NumMatrix::Identity(3, 3), dt * NumMatrix::Identity(3, 3),
            NumMatrix::Zero(3, 13), NumMatrix::Identity(3, 3);

    return F;
}

NumMatrix QuaternionKalman::create_proc_noise_cov_mtx(double dt) const
{
    /* useful constants */
    double dt_sq = dt * dt;
    double dt_2 = dt / 2;
    double dt_sq_2 = dt_sq / 2;

    NumMatrix K = qutils::quat_delta_mtx(get_orientation_quaternion(), dt_2);

    NumMatrix Qq = params.proc_params.gyro_std * params.proc_params.gyro_std * K * K.transpose();
    NumMatrix Qb = params.proc_params.gyro_bias_std * params.proc_params.gyro_bias_std * NumMatrix::Identity(3, 3);

    NumMatrix G(9, 3);
    G << NumMatrix::Identity(3, 3) * dt_sq_2,
            NumMatrix::Identity(3, 3) * dt,
            NumMatrix::Identity(3, 3);

    NumMatrix Qp = params.proc_params.accel_std * params.proc_params.accel_std * G * G.transpose();

    NumMatrix Q(state_size, state_size);

    Q << Qq, NumMatrix::Zero(4, 12),
            NumMatrix::Zero(3, 4), Qb, NumMatrix::Zero(3, 9),
            NumMatrix::Zero(9, 7), Qp;

    return Q;
}

NumMatrix QuaternionKalman::create_meas_noise_cov_mtx(double lat, double lon, double alt, QDate day) const
{
    NumMatrix Ra = params.meas_params.accel_std * params.meas_params.accel_std * NumMatrix::Identity(3, 3);

    double mag_magn = WrapperWMM::instance().expected_mag_magnitude(lat, lon, alt, day);
    double normalized_magn_std = params.meas_params.magn_std / mag_magn;
    NumMatrix Rm = normalized_magn_std * normalized_magn_std * NumMatrix::Identity(3, 3);

    double horizontal_linear_std = params.meas_params.gps_cep * 1.2;
    double altitude_std = horizontal_linear_std / 0.53;

    NumMatrix Cel = WrapperWMM::instance().geodetic_to_dcm(lat, lon);
    NumMatrix local_cov = NumMatrix::Identity(3, 3);
    local_cov(0, 0) = horizontal_linear_std * horizontal_linear_std;
    local_cov(1, 1) = horizontal_linear_std * horizontal_linear_std;
    local_cov(2, 2) = altitude_std * altitude_std;

    NumMatrix Rp = Cel.transpose() * local_cov * Cel;

    NumMatrix R(measurement_size, measurement_size);

    R << Ra, NumMatrix::Zero(3, 7),
            NumMatrix::Zero(3, 3), Rm, NumMatrix::Zero(3, 4),
            NumMatrix::Zero(3, 6), Rp, NumMatrix::Zero(3, 1),
            NumMatrix::Zero(1, 9), params.meas_params.gps_vel_abs_std * params.meas_params.gps_vel_abs_std;

    return R;
}

NumMatrix QuaternionKalman::create_meas_proj_mtx(double lat, double lon, double alt, QDate day, const NumVector & v) const
{
    // 1
    NumMatrix Dac_Dq(3, 4);

    double height_adjust = WrapperWMM::instance().expected_gravity_accel(lat, alt) / phconst::standard_gravity;

    NumVector a = get_acceleration() / phconst::standard_gravity;
    NumVector q = get_orientation_quaternion();

    NumMatrix Cel = WrapperWMM::instance().geodetic_to_dcm(lat, lon);
    NumMatrix Clb = qutils::quaternion_to_dcm_tr(q);
    NumMatrix Ceb = Clb * Cel;

    NumMatrix Ddcm_Dqs = qutils::ddcm_dqs_tr(q);
    NumMatrix Ddcm_Dqx = qutils::ddcm_dqx_tr(q);
    NumMatrix Ddcm_Dqy = qutils::ddcm_dqy_tr(q);
    NumMatrix Ddcm_Dqz = qutils::ddcm_dqz_tr(q);

    NumVector tmp = Cel * a;

    NumVector col_s = Ddcm_Dqs.col(2) * height_adjust + Ddcm_Dqs * tmp;
    NumVector col_x = Ddcm_Dqx.col(2) * height_adjust + Ddcm_Dqx * tmp;
    NumVector col_y = Ddcm_Dqy.col(2) * height_adjust + Ddcm_Dqy * tmp;
    NumVector col_z = Ddcm_Dqz.col(2) * height_adjust + Ddcm_Dqz * tmp;

    Dac_Dq << col_s, col_x, col_y, col_z;

    // 2
    NumMatrix Dac_Dpos(3, 3);

    NumMatrix Dgeo_Dpos = WrapperWMM::instance().dgeo_dpos(lat, lon, alt);
    NumMatrix Ddcm_Dlat = WrapperWMM::instance().dcm_lat_partial(lat, lon);
    NumMatrix Ddcm_Dlon = WrapperWMM::instance().dcm_lon_partial(lat, lon);

    NumMatrix Dcel_Dx = Dgeo_Dpos(0, 0) * Ddcm_Dlat + Dgeo_Dpos(1, 0) * Ddcm_Dlon;
    NumMatrix Dcel_Dy = Dgeo_Dpos(0, 1) * Ddcm_Dlat + Dgeo_Dpos(1, 1) * Ddcm_Dlon;
    NumMatrix Dcel_Dz = Dgeo_Dpos(0, 2) * Ddcm_Dlat + Dgeo_Dpos(1, 2) * Ddcm_Dlon;

    col_x = Clb * Dcel_Dx * a;
    col_y = Clb * Dcel_Dy * a;
    col_z = Clb * Dcel_Dz * a;

    Dac_Dpos << col_x, col_y, col_z;

    // 3
    NumMatrix Dac_Da = Ceb / phconst::standard_gravity;

    // 4
    NumMatrix Dm_Dq(3, 4);

    NumVector earth_mag = WrapperWMM::instance().expected_mag(lat, lon, alt, day);

    col_s = Ddcm_Dqs * earth_mag;
    col_x = Ddcm_Dqx * earth_mag;
    col_y = Ddcm_Dqy * earth_mag;
    col_z = Ddcm_Dqz * earth_mag;

    Dm_Dq << col_s, col_x, col_y, col_z;

    // 5
    NumMatrix Dpos_Dpos = NumMatrix::Identity(3, 3);

    // 6
    NumMatrix Dv_Dv(1, 3);
    double v_abs = v.norm();

    if(v_abs > 0)
    {
        Dv_Dv << v[0] / v_abs, v[1] / v_abs, v[2] / v_abs;
    }
    else
    {
        Dv_Dv << 0, 0, 0;
    }

    // Combine
    NumMatrix H(measurement_size, state_size);
    H <<    Dac_Dq, NumMatrix::Zero(3, 3), Dac_Dpos, NumMatrix::Zero(3, 3), Dac_Da,
            Dm_Dq, NumMatrix::Zero(3, 12),
            NumMatrix::Zero(3, 7), Dpos_Dpos, NumMatrix::Zero(3, 6),
            NumMatrix::Zero(1, 10), Dv_Dv, NumMatrix::Zero(1, 3);

    return H;
}

void QuaternionKalman::calculate_geodetic(const NumVector & position,
                                          double & lat, double & lon, double & alt) const
{
    WrapperWMM::instance().cartesian_to_geodetic(position, lat, lon, alt);
}

void QuaternionKalman::calculate_accelerometer(const NumVector & orientation_quat, const NumVector & acceleration,
                             double lat, double lon, double alt,
                             double & ax, double & ay, double & az) const
{
    double height_adjust = WrapperWMM::instance().expected_gravity_accel(lat, alt) / phconst::standard_gravity;

    NumMatrix Clb = qutils::quaternion_to_dcm_tr(orientation_quat);
    NumMatrix Cel = WrapperWMM::instance().geodetic_to_dcm(lat, lon);

    NumVector movement_component = Clb * Cel * acceleration / phconst::standard_gravity;
    NumVector g(3);
    g << 0, 0, height_adjust;

    NumVector gravity_component = Clb * g;

    NumVector rot_accel = gravity_component + movement_component;

    ax = rot_accel(0);
    ay = rot_accel(1);
    az = rot_accel(2);
}

void QuaternionKalman::calculate_magnetometer(const NumVector & orientation_quat,
                                              double lat, double lon, double alt, QDate day,
                                              double & mx, double & my, double & mz) const
{
    NumVector rot_magn = qutils::quaternion_to_dcm_tr(orientation_quat) *
                            WrapperWMM::instance().expected_mag(lat, lon, alt, day);

    mx = rot_magn(0);
    my = rot_magn(1);
    mz = rot_magn(2);
}

void QuaternionKalman::calculate_velocity(const NumVector & velocity, double & vel) const
{
    vel = velocity.norm();
}

NumVector QuaternionKalman::get_orientation_quaternion() const
{
    return x.segment(0, 4);
}

NumVector QuaternionKalman::get_gyro_bias() const
{
    return x.segment(4, 3);
}

NumVector QuaternionKalman::get_position() const
{
    return x.segment(7, 3);
}

NumVector QuaternionKalman::get_velocity() const
{
    return x.segment(10, 3);
}

NumVector QuaternionKalman::get_acceleration() const
{
    return x.segment(13, 3);
}

void QuaternionKalman::get_rpy(double & roll, double & pitch, double & yaw) const
{
    qutils::quat_to_rpy(get_orientation_quaternion(), roll, pitch, yaw);
}

void QuaternionKalman::get_geodetic(double & lat, double & lon, double & alt) const
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

void QuaternionKalman::set_init_qs_std(double std)
{
    params.init_params.qs_std = std;
}

void QuaternionKalman::set_init_qx_std(double std)
{
    params.init_params.qx_std = std;
}

void QuaternionKalman::set_init_qy_std(double std)
{
    params.init_params.qy_std = std;
}

void QuaternionKalman::set_init_qz_std(double std)
{
    params.init_params.qz_std = std;
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
