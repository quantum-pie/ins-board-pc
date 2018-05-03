#include "orientationekf.h"
#include "wmmwrapper.h"
#include "physconst.h"

#include <Eigen/Dense>

#include <QtMath>

const int OrientationEKF::state_size = 7;
const int OrientationEKF::measurement_size = 6;

OrientationEKF::OrientationEKF(const FilterParams & par)
    : KalmanOrientationFilter(par.accum_capacity),
      params(par)
{
    x = NumVector(state_size);
    P = NumMatrix(state_size, state_size);
}

OrientationEKF::~OrientationEKF()
{

}

void OrientationEKF::initialize(const FilterInput & z)
{
    KalmanOrientationFilter::initialize(z);

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

void OrientationEKF::step(const FilterInput & z)
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

void OrientationEKF::update(const FilterInput & z)
{
    NumMatrix F = create_transition_mtx(z);
    NumMatrix Q = create_proc_noise_cov_mtx(z.dt);

    x = F * x;
    normalize_state();

    P = F * P * F.transpose() + Q;

    NumVector predicted_orientation = get_orientation_quaternion();
    NumVector pred_acc = calculate_accelerometer(predicted_orientation, z.geo);
    NumVector pred_magn = calculate_magnetometer(predicted_orientation, z.geo, z.day);

    NumVector z_pr(measurement_size);
    z_pr << pred_acc, pred_magn;

    NumVector z_meas(measurement_size);
    z_meas << z.a, z.m;

    NumVector y = z_meas - z_pr;

    NumMatrix R = create_meas_noise_cov_mtx(z.geo, z.day);
    NumMatrix H = create_meas_proj_mtx(z.geo, z.day);

    NumMatrix S = H * P * H.transpose() + R;
    NumMatrix K = P * H.transpose() * S.inverse();

    x += K * y;
    normalize_state();

    NumMatrix tmp = NumMatrix::Identity(x.size(), x.size()) - K * H;
    P = tmp * P * tmp.transpose() + K * R * K.transpose();
}

void OrientationEKF::normalize_state()
{
    x.segment(0, 4) = qutils::quat_normalize(get_orientation_quaternion());
}

NumMatrix OrientationEKF::create_transition_mtx(const FilterInput & z) const
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

NumMatrix OrientationEKF::create_proc_noise_cov_mtx(double dt) const
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

NumMatrix OrientationEKF::create_meas_noise_cov_mtx(const NumVector & geo, QDate day) const
{
    NumMatrix Ra = params.meas_params.accel_std * params.meas_params.accel_std * NumMatrix::Identity(3, 3);

    double mag_magn = WrapperWMM::instance().expected_mag_magnitude(geo, day);
    double normalized_magn_std = params.meas_params.magn_std / mag_magn;
    NumMatrix Rm = normalized_magn_std * normalized_magn_std * NumMatrix::Identity(3, 3);

    NumMatrix R(measurement_size, measurement_size);

    R << Ra, NumMatrix::Zero(3, 3),
         NumMatrix::Zero(3, 3), Rm;

    return R;
}

NumMatrix OrientationEKF::create_meas_proj_mtx(const NumVector & geo, QDate day) const
{
    // 1
    NumMatrix Dac_Dq(3, 4);

    double height_adjust = WrapperWMM::instance().expected_gravity_accel(geo) / phconst::standard_gravity;

    NumVector q = get_orientation_quaternion();

    NumMatrix Ddcm_Dqs = qutils::ddcm_dqs_tr(q);
    NumMatrix Ddcm_Dqx = qutils::ddcm_dqx_tr(q);
    NumMatrix Ddcm_Dqy = qutils::ddcm_dqy_tr(q);
    NumMatrix Ddcm_Dqz = qutils::ddcm_dqz_tr(q);

    NumVector col_s = Ddcm_Dqs.col(2) * height_adjust;
    NumVector col_x = Ddcm_Dqx.col(2) * height_adjust;
    NumVector col_y = Ddcm_Dqy.col(2) * height_adjust;
    NumVector col_z = Ddcm_Dqz.col(2) * height_adjust;

    Dac_Dq << col_s, col_x, col_y, col_z;

    // 2
    NumMatrix Dm_Dq(3, 4);

    NumVector earth_mag = WrapperWMM::instance().expected_mag(geo, day);

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

NumVector OrientationEKF::calculate_accelerometer(const NumVector & orientation_quat,
                             const NumVector & geo) const
{
    double height_adjust = WrapperWMM::instance().expected_gravity_accel(geo) / phconst::standard_gravity;

    NumMatrix Clb = qutils::quaternion_to_dcm_tr(orientation_quat);

    NumVector g(3);
    g << 0, 0, height_adjust;

    return Clb * g;
}

NumVector OrientationEKF::calculate_magnetometer(const NumVector & orientation_quat,
                                              const NumVector & geo, QDate day) const
{
    return qutils::quaternion_to_dcm_tr(orientation_quat) *
                            WrapperWMM::instance().expected_mag(geo, day);
}

NumVector OrientationEKF::get_orientation_quaternion() const
{
    return x.segment(0, 4);
}

NumVector OrientationEKF::get_gyro_bias() const
{
    return x.segment(4, 3);
}

void OrientationEKF::set_proc_gyro_std(double std)
{
    params.proc_params.gyro_std = std;
}

void OrientationEKF::set_proc_gyro_bias_std(double std)
{
    params.proc_params.gyro_bias_std = std;
}

void OrientationEKF::set_meas_accel_std(double std)
{
    params.meas_params.accel_std = std;
}

void OrientationEKF::set_meas_magn_std(double std)
{
    params.meas_params.magn_std = std;
}

void OrientationEKF::set_init_qs_std(double std)
{
    params.init_params.qs_std = std;
}

void OrientationEKF::set_init_qx_std(double std)
{
    params.init_params.qx_std = std;
}

void OrientationEKF::set_init_qy_std(double std)
{
    params.init_params.qy_std = std;
}

void OrientationEKF::set_init_qz_std(double std)
{
    params.init_params.qz_std = std;
}

void OrientationEKF::set_init_bias_std(double std)
{
    params.init_params.bias_std = std;
}
