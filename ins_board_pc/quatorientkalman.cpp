#include "quatorientkalman.h"
#include "wmmwrapper.h"
#include "physconst.h"
#include "quaternions.h"

#include <Eigen/Dense>

#include <QtMath>
#include <QDebug>

const int QuaternionOrientationKalman::state_size = 7;
const int QuaternionOrientationKalman::measurement_size = 6;

QuaternionOrientationKalman::QuaternionOrientationKalman(const FilterParams & par)
    : AbstractOrientationFilter(par.accum_capacity),
      params(par)
{
    x = NumVector(state_size);
    P = NumMatrix(state_size, state_size);
}

QuaternionOrientationKalman::~QuaternionOrientationKalman()
{

}

void QuaternionOrientationKalman::accumulate(const FilterInput & z)
{
    AbstractOrientationFilter::accumulate(z);
}

void QuaternionOrientationKalman::reset()
{
    AbstractOrientationFilter::reset();
}

void QuaternionOrientationKalman::initialize(const FilterInput & z)
{
    AbstractOrientationFilter::initialize(z);

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

    P = NumMatrix::Zero(state_size, state_size);
    P(0, 0) = params.init_params.qs_std * params.init_params.qs_std;
    P(1, 1) = params.init_params.qx_std * params.init_params.qx_std;
    P(2, 2) = params.init_params.qy_std * params.init_params.qy_std;
    P(3, 3) = params.init_params.qz_std * params.init_params.qz_std;

    double bias_var = params.init_params.bias_std * params.init_params.bias_std;
    P(4, 4) = bias_var;
    P(5, 5) = bias_var;
    P(6, 6) = bias_var;
}

void QuaternionOrientationKalman::step(const FilterInput & z)
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

void QuaternionOrientationKalman::update(const FilterInput & z)
{
    NumMatrix F = create_transition_mtx(z);
    NumMatrix Q = create_proc_noise_cov_mtx(z.dt);

    x = F * x;
    normalize_state();

    P = F * P * F.transpose() + Q;

    NumVector z_pr(measurement_size);

    double lat, lon, alt;
    lat = z.geo[0];
    lon = z.geo[1];
    alt = z.geo[2];

    NumVector predicted_orientation = get_orientation_quaternion();

    calculate_accelerometer(predicted_orientation, lat, alt, z_pr(0), z_pr(1), z_pr(2));
    calculate_magnetometer(predicted_orientation, lat, lon, alt, z.day, z_pr(3), z_pr(4), z_pr(5));

    NumVector z_meas(measurement_size);
    z_meas << z.a, z.m;

    NumVector y = z_meas - z_pr;

    NumMatrix R = create_meas_noise_cov_mtx(lat, lon, alt, z.day);
    NumMatrix H = create_meas_proj_mtx(lat, lon, alt, z.day);

    NumMatrix S = H * P * H.transpose() + R;
    NumMatrix K = P * H.transpose() * S.inverse();

    x += K * y;
    normalize_state();

    // TODO: Implement optimal formula
    P = (NumMatrix::Identity(x.size(), x.size()) - K * H) * P;
}

void QuaternionOrientationKalman::normalize_state()
{
    x.segment(0, 4) = qutils::quat_normalize(get_orientation_quaternion());
}

NumMatrix QuaternionOrientationKalman::create_transition_mtx(const FilterInput & z) const
{
    /* useful constants */
    double dt_2 = z.dt / 2;

    /* constructing state transition matrix */
    NumMatrix V = qutils::skew_symmetric(z.w);

    V *= dt_2;
    V += NumMatrix::Identity(4, 4);

    NumMatrix K = qutils::quat_delta_mtx(get_orientation_quaternion(), dt_2);

    NumMatrix F(state_size, state_size);

    F << V, K,
         NumMatrix::Zero(3, 4), NumMatrix::Identity(3, 3);

    return F;
}

NumMatrix QuaternionOrientationKalman::create_proc_noise_cov_mtx(double dt) const
{
    /* useful constants */
    double dt_2 = dt / 2;

    NumMatrix K = qutils::quat_delta_mtx(get_orientation_quaternion(), dt_2);

    NumMatrix Qq = params.proc_params.gyro_std * params.proc_params.gyro_std * K * K.transpose();
    NumMatrix Qb = params.proc_params.gyro_bias_std * params.proc_params.gyro_bias_std * NumMatrix::Identity(3, 3);

    NumMatrix Q(state_size, state_size);

    Q << Qq, NumMatrix::Zero(4, 3),
            NumMatrix::Zero(3, 4), Qb;

    return Q;
}

NumMatrix QuaternionOrientationKalman::create_meas_noise_cov_mtx(double lat, double lon, double alt, QDate day) const
{
    NumMatrix Ra = params.meas_params.accel_std * params.meas_params.accel_std * NumMatrix::Identity(3, 3);

    double mag_magn = WrapperWMM::instance().expected_mag_magnitude(lat, lon, alt, day);
    double normalized_magn_std = params.meas_params.magn_std / mag_magn;
    NumMatrix Rm = normalized_magn_std * normalized_magn_std * NumMatrix::Identity(3, 3);

    NumMatrix R(measurement_size, measurement_size);

    R << Ra, NumMatrix::Zero(3, 3),
         NumMatrix::Zero(3, 3), Rm;

    return R;
}

NumMatrix QuaternionOrientationKalman::create_meas_proj_mtx(double lat, double lon, double alt, QDate day) const
{
    // 1
    NumMatrix Dac_Dq(3, 4);

    double height_adjust = WrapperWMM::instance().expected_gravity_accel(lat, alt) / phconst::standard_gravity;

    NumVector q = get_orientation_quaternion();

    NumMatrix Clb = qutils::quaternion_to_dcm_tr(q);

    NumMatrix Ddcm_Dqs = qutils::ddcm_dqs_tr(q);
    NumMatrix Ddcm_Dqx = qutils::ddcm_dqx_tr(q);
    NumMatrix Ddcm_Dqy = qutils::ddcm_dqy_tr(q);
    NumMatrix Ddcm_Dqz = qutils::ddcm_dqz_tr(q);

    NumVector col_s = Ddcm_Dqs.col(2) * height_adjust;
    NumVector col_x = Ddcm_Dqx.col(2) * height_adjust;
    NumVector col_y = Ddcm_Dqy.col(2) * height_adjust;
    NumVector col_z = Ddcm_Dqz.col(2) * height_adjust;

    Dac_Dq << col_s, col_x, col_y, col_z;

    // 4
    NumMatrix Dm_Dq(3, 4);

    NumVector earth_mag = WrapperWMM::instance().expected_mag(lat, lon, alt, day);

    col_s = Ddcm_Dqs * earth_mag;
    col_x = Ddcm_Dqx * earth_mag;
    col_y = Ddcm_Dqy * earth_mag;
    col_z = Ddcm_Dqz * earth_mag;

    Dm_Dq << col_s, col_x, col_y, col_z;


    // Combine
    NumMatrix H(measurement_size, state_size);
    H <<    Dac_Dq, NumMatrix::Zero(3, 3),
            Dm_Dq, NumMatrix::Zero(3, 3);

    return H;
}

void QuaternionOrientationKalman::calculate_accelerometer(const NumVector & orientation_quat,
                             double lat, double alt,
                             double & ax, double & ay, double & az) const
{
    double height_adjust = WrapperWMM::instance().expected_gravity_accel(lat, alt) / phconst::standard_gravity;

    NumMatrix Clb = qutils::quaternion_to_dcm_tr(orientation_quat);

    NumVector g(3);
    g << 0, 0, height_adjust;

    NumVector gravity_component = Clb * g;

    NumVector rot_accel = gravity_component;

    ax = rot_accel(0);
    ay = rot_accel(1);
    az = rot_accel(2);
}

void QuaternionOrientationKalman::calculate_magnetometer(const NumVector & orientation_quat,
                                              double lat, double lon, double alt, QDate day,
                                              double & mx, double & my, double & mz) const
{
    NumVector rot_magn = qutils::quaternion_to_dcm_tr(orientation_quat) *
                            WrapperWMM::instance().expected_mag(lat, lon, alt, day);

    mx = rot_magn(0);
    my = rot_magn(1);
    mz = rot_magn(2);
}

NumVector QuaternionOrientationKalman::get_orientation_quaternion() const
{
    return x.segment(0, 4);
}

NumVector QuaternionOrientationKalman::get_gyro_bias() const
{
    return x.segment(4, 3);
}

void QuaternionOrientationKalman::get_rpy(double & roll, double & pitch, double & yaw) const
{
    qutils::quat_to_rpy(get_orientation_quaternion(), roll, pitch, yaw);
}

void QuaternionOrientationKalman::set_proc_gyro_std(double std)
{
    params.proc_params.gyro_std = std;
}

void QuaternionOrientationKalman::set_proc_gyro_bias_std(double std)
{
    params.proc_params.gyro_bias_std = std;
}

void QuaternionOrientationKalman::set_meas_accel_std(double std)
{
    params.meas_params.accel_std = std;
}

void QuaternionOrientationKalman::set_meas_magn_std(double std)
{
    params.meas_params.magn_std = std;
}

void QuaternionOrientationKalman::set_init_qs_std(double std)
{
    params.init_params.qs_std = std;
}

void QuaternionOrientationKalman::set_init_qx_std(double std)
{
    params.init_params.qx_std = std;
}

void QuaternionOrientationKalman::set_init_qy_std(double std)
{
    params.init_params.qy_std = std;
}

void QuaternionOrientationKalman::set_init_qz_std(double std)
{
    params.init_params.qz_std = std;
}

void QuaternionOrientationKalman::set_init_bias_std(double std)
{
    params.init_params.bias_std = std;
}
