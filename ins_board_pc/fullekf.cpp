#include "fullekf.h"
#include "physconst.h"

#include <Eigen/Dense>

#include <QtMath>

#include <QDebug>

const int FullEKF::state_size = 16;
const int FullEKF::measurement_size = 12;

FullEKF::FullEKF(const FilterParams & par)
    : KalmanOrientationFilter(par.accum_capacity),
      KalmanPositionFilter(par.track_history),
      params(par)
{
    x = NumVector(state_size);
    P = NumMatrix(state_size, state_size);
}

FullEKF::~FullEKF()
{

}

void FullEKF::accumulate(const FilterInput & z)
{
    KalmanOrientationFilter::accumulate(z);
    KalmanPositionFilter::accumulate(z);
}

void FullEKF::reset()
{
    KalmanOrientationFilter::reset();
    KalmanPositionFilter::reset();
}

void FullEKF::initialize(const FilterInput & z)
{
    KalmanOrientationFilter::initialize(z);
    KalmanPositionFilter::initialize(z);

    NumVector qacc = qutils::acceleration_quat(z.a);

    NumMatrix accel_rotator = qutils::quaternion_to_dcm_tr(qacc);
    NumVector l = accel_rotator * z.m;

    NumVector qmag = qutils::magnetometer_quat(l);

    double declination, inclination, magn;
    WrapperWMM::instance().measure(z.geo, z.day, declination, inclination, magn);

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

void FullEKF::step(const FilterInput & z)
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

void FullEKF::update(const FilterInput & z)
{
    NumMatrix F = create_transition_mtx(z);
    NumMatrix Q = create_proc_noise_cov_mtx(z.dt);

    x = F * x;
    normalize_state();

    P = F * P * F.transpose() + Q;

    if(z.gps_fresh)
    {
        NumVector predicted_pos = get_position();
        NumVector predicted_orientation = get_orientation_quaternion();

        NumVector geo = calculate_geodetic(predicted_pos);

        NumVector predicted_acc = calculate_accelerometer(predicted_orientation, get_acceleration(), geo);
        NumVector predicted_magn = calculate_magnetometer(predicted_orientation, geo, z.day);

        NumVector z_pr(measurement_size);
        z_pr << predicted_acc, predicted_magn, predicted_pos, get_velocity();

        NumVector z_meas(measurement_size);
        z_meas << z.a, z.m, z.pos, z.v;

        //eaux::debug_vector(z_meas, "z meas");
        //eaux::debug_vector(z_pr, "z pred");

        NumVector y = z_meas - z_pr;

        NumMatrix R = create_meas_noise_cov_mtx(geo, z.day);
        NumMatrix H = create_meas_proj_mtx(geo, z.day);

        NumMatrix S = H * P * H.transpose() + R;
        NumMatrix K = P * H.transpose() * S.inverse();

        x += K * y;
        normalize_state();

        NumMatrix tmp = NumMatrix::Identity(x.size(), x.size()) - K * H;
        P = tmp * P * tmp.transpose() + K * R * K.transpose();

        KalmanPositionFilter::update(z);
    }
}

void FullEKF::normalize_state()
{
    x.segment(0, 4) = qutils::quat_normalize(get_orientation_quaternion());
}

NumMatrix FullEKF::create_transition_mtx(const FilterInput & z) const
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

NumMatrix FullEKF::create_proc_noise_cov_mtx(double dt) const
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

NumMatrix FullEKF::create_meas_noise_cov_mtx(const NumVector & geo, QDate day) const
{
    NumMatrix Ra = params.meas_params.accel_std * params.meas_params.accel_std * NumMatrix::Identity(3, 3);

    double mag_magn = WrapperWMM::instance().expected_mag_magnitude(geo, day);
    double normalized_magn_std = params.meas_params.magn_std / mag_magn;
    NumMatrix Rm = normalized_magn_std * normalized_magn_std * NumMatrix::Identity(3, 3);

    double horizontal_linear_std = params.meas_params.gps_cep * 1.2;
    double altitude_std = horizontal_linear_std / 0.53;

    NumMatrix Cel = WrapperWMM::instance().geodetic_to_dcm(geo);
    NumMatrix local_cov = NumMatrix::Identity(3, 3);
    local_cov(0, 0) = horizontal_linear_std * horizontal_linear_std;
    local_cov(1, 1) = horizontal_linear_std * horizontal_linear_std;
    local_cov(2, 2) = altitude_std * altitude_std;

    NumMatrix Rp = Cel.transpose() * local_cov * Cel;

    NumMatrix R(measurement_size, measurement_size);

    double vel_variance = params.meas_params.gps_vel_std * params.meas_params.gps_vel_std;
    R << Ra, NumMatrix::Zero(3, 9),
            NumMatrix::Zero(3, 3), Rm, NumMatrix::Zero(3, 6),
            NumMatrix::Zero(3, 6), Rp, NumMatrix::Zero(3, 3),
            NumMatrix::Zero(3, 9), NumMatrix::Identity(3, 3) * vel_variance;

    return R;
}

NumMatrix FullEKF::create_meas_proj_mtx(const NumVector & geo, QDate day) const
{
    // 1
    NumMatrix Dac_Dq(3, 4);

    double height_adjust = WrapperWMM::instance().expected_gravity_accel(geo) / phconst::standard_gravity;

    NumVector a = get_acceleration() / phconst::standard_gravity;
    NumVector q = get_orientation_quaternion();

    NumMatrix Cel = WrapperWMM::instance().geodetic_to_dcm(geo);
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

    NumMatrix Dgeo_Dpos = WrapperWMM::instance().dgeo_dpos(geo);
    NumMatrix Ddcm_Dlat = WrapperWMM::instance().dcm_lat_partial(geo);
    NumMatrix Ddcm_Dlon = WrapperWMM::instance().dcm_lon_partial(geo);

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

    NumVector earth_mag = WrapperWMM::instance().expected_mag(geo, day);

    col_s = Ddcm_Dqs * earth_mag;
    col_x = Ddcm_Dqx * earth_mag;
    col_y = Ddcm_Dqy * earth_mag;
    col_z = Ddcm_Dqz * earth_mag;

    Dm_Dq << col_s, col_x, col_y, col_z;

    // 5
    NumMatrix Dpos_Dpos = NumMatrix::Identity(3, 3);

    // 6
    NumMatrix Dv_Dv = NumMatrix::Identity(3, 3);

    // Combine
    NumMatrix H(measurement_size, state_size);
    H <<    Dac_Dq, NumMatrix::Zero(3, 3), Dac_Dpos, NumMatrix::Zero(3, 3), Dac_Da,
            Dm_Dq, NumMatrix::Zero(3, 12),
            NumMatrix::Zero(3, 7), Dpos_Dpos, NumMatrix::Zero(3, 6),
            NumMatrix::Zero(3, 10), Dv_Dv, NumMatrix::Zero(3, 3);

    return H;
}

NumVector FullEKF::calculate_accelerometer(const NumVector & orientation_quat, const NumVector & acceleration,
                             const NumVector & geo) const
{
    double height_adjust = WrapperWMM::instance().expected_gravity_accel(geo) / phconst::standard_gravity;

    NumMatrix Clb = qutils::quaternion_to_dcm_tr(orientation_quat);
    NumMatrix Cel = WrapperWMM::instance().geodetic_to_dcm(geo);

    NumVector movement_component = Clb * Cel * acceleration / phconst::standard_gravity;
    NumVector g(3);
    g << 0, 0, height_adjust;

    NumVector gravity_component = Clb * g;

    return gravity_component + movement_component;
}

NumVector FullEKF::calculate_magnetometer(const NumVector & orientation_quat,
                                              const NumVector & geo, QDate day) const
{
    return qutils::quaternion_to_dcm_tr(orientation_quat) *
                            WrapperWMM::instance().expected_mag(geo, day);
}

NumVector FullEKF::get_orientation_quaternion() const
{
    return x.segment(0, 4);
}

NumVector FullEKF::get_gyro_bias() const
{
    return x.segment(4, 3);
}

NumVector FullEKF::get_position() const
{
    return x.segment(7, 3);
}

NumVector FullEKF::get_velocity() const
{
    return x.segment(10, 3);
}

NumVector FullEKF::get_acceleration() const
{
    return x.segment(13, 3);
}

void FullEKF::set_proc_gyro_std(double std)
{
    params.proc_params.gyro_std = std;
}

void FullEKF::set_proc_gyro_bias_std(double std)
{
    params.proc_params.gyro_bias_std = std;
}

void FullEKF::set_proc_accel_std(double std)
{
    params.proc_params.accel_std = std;
}

void FullEKF::set_meas_accel_std(double std)
{
    params.meas_params.accel_std = std;
}

void FullEKF::set_meas_magn_std(double std)
{
    params.meas_params.magn_std = std;
}

void FullEKF::set_meas_pos_std(double std)
{
    params.meas_params.gps_cep = std;
}

void FullEKF::set_meas_vel_std(double std)
{
    params.meas_params.gps_vel_std = std;
}

void FullEKF::set_init_qs_std(double std)
{
    params.init_params.qs_std = std;
}

void FullEKF::set_init_qx_std(double std)
{
    params.init_params.qx_std = std;
}

void FullEKF::set_init_qy_std(double std)
{
    params.init_params.qy_std = std;
}

void FullEKF::set_init_qz_std(double std)
{
    params.init_params.qz_std = std;
}

void FullEKF::set_init_bias_std(double std)
{
    params.init_params.bias_std = std;
}

void FullEKF::set_init_pos_std(double std)
{
    params.init_params.pos_std = std;
}

void FullEKF::set_init_vel_std(double std)
{
    params.init_params.vel_std = std;
}

void FullEKF::set_init_accel_std(double std)
{
    params.init_params.accel_std = std;
}
