#include "fullukf.h"
#include "physconst.h"

#include <Eigen/Dense>
#include <QtMath>
#include <QVector>

#include <QDebug>

const int FullUKF::state_size = 16;
const int FullUKF::measurement_size = 12;

FullUKF::FullUKF(const FilterParams & par)
    : KalmanOrientationFilter(par.accum_capacity),
      KalmanPositionFilter(par.track_history),
      params(par), L(state_size),
      Ws(2 * L + 1), Wc(2 * L + 1)
{
    x = NumVector(state_size);
    P = NumMatrix(state_size, state_size);

    double alpha_sq = par.ut_params.alpha * par.ut_params.alpha;
    lambda = alpha_sq * (L + par.ut_params.kappa) - L;

    Ws[0] = lambda / (L + lambda);
    Wc[0] = Ws[0] + (1 - alpha_sq + par.ut_params.beta);

    for(int i = 1; i <= 2 * L; ++i)
    {
        Ws[i] = 0.5 / (L + lambda);
        Wc[i] = 0.5 / (L + lambda);
    }
}

FullUKF::~FullUKF()
{

}

void FullUKF::accumulate(const FilterInput & z)
{
    KalmanOrientationFilter::accumulate(z);
    KalmanPositionFilter::accumulate(z);
}

void FullUKF::reset()
{
    KalmanOrientationFilter::reset();
    KalmanPositionFilter::reset();
}

void FullUKF::initialize(const FilterInput & z)
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

    // TODO: init properly with 2 samples accumulation
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

void FullUKF::step(const FilterInput & z)
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

void FullUKF::update(const FilterInput & z)
{
    // predict (EKF version)
    NumMatrix F = create_transition_mtx(z);
    NumMatrix Q = create_proc_noise_cov_mtx(z.dt);

    x = F * x;
    normalize_state();

    P = F * P * F.transpose() + Q;

    if(z.gps_fresh)
    {
        // update (unscented version)
        QVector<NumVector> sigma_p(2 * L + 1);
        sigma_p[0] = x;

        NumMatrix mtx_root = P.llt().matrixL();
        mtx_root *= qSqrt(L + lambda);

        for(int i = 1; i <= L; ++i)
        {
            sigma_p[i] = x + mtx_root.col(i - 1);
            sigma_p[i + L] = x - mtx_root.col(i - 1);
        }

        QVector<NumVector> sigma_z(2 * L + 1);
        NumVector z_p = NumMatrix::Zero(measurement_size, 1);

        NumVector sigma_quat(4);
        NumVector sigma_accel(3);
        NumVector sigma_pos(3);
        NumVector sigma_vel(3);

        NumVector geo(3);
        NumVector pred_acc(3);
        NumVector pred_magn(3);

        for(int i = 0; i <= 2 * L; ++i)
        {
            x = sigma_p[i];

            sigma_quat = get_orientation_quaternion();
            sigma_accel = get_acceleration();
            sigma_pos = get_position();
            sigma_vel = get_velocity();

            geo = calculate_geodetic(sigma_pos);
            pred_acc = calculate_accelerometer(sigma_quat, sigma_accel, geo);
            pred_magn = calculate_magnetometer(sigma_quat, geo, z.day);

            NumVector z_pr(measurement_size);
            z_pr << pred_acc, pred_magn, sigma_pos, sigma_vel;

            sigma_z[i] = z_pr;
            z_p += Ws[i] * z_pr;
        }

        x = sigma_p[0];

        NumMatrix Pzz = NumMatrix::Zero(measurement_size, measurement_size);
        NumMatrix Pxz = NumMatrix::Zero(state_size, measurement_size);

        for(int i = 0; i <= 2 * L; ++i)
        {
            Pzz += Wc[i] * (sigma_z[i] - z_p) * (sigma_z[i] - z_p).transpose();
            Pxz += Wc[i] * (sigma_p[i] - x ) * (sigma_z[i] - z_p).transpose();
        }

        NumMatrix R = create_meas_noise_cov_mtx(z.geo, z.day);
        Pzz += R;

        NumMatrix K = Pxz * Pzz.inverse();

        NumVector z_meas(measurement_size);
        z_meas << z.a, z.m, z.pos, z.v;

        x += K * (z_meas - z_p);
        normalize_state();

        P -= K * Pzz * K.transpose();

        KalmanPositionFilter::update(z);
    }
}

void FullUKF::normalize_state()
{
    x.segment(0, 4) = qutils::quat_normalize(get_orientation_quaternion());
}

NumMatrix FullUKF::create_transition_mtx(const FilterInput & z) const
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

NumMatrix FullUKF::create_proc_noise_cov_mtx(double dt) const
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

NumMatrix FullUKF::create_meas_noise_cov_mtx(const NumVector & geo, QDate day) const
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

NumVector FullUKF::calculate_accelerometer(const NumVector & orientation_quat, const NumVector & acceleration,
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

NumVector FullUKF::calculate_magnetometer(const NumVector & orientation_quat,
                                              const NumVector & geo, QDate day) const
{
    return qutils::quaternion_to_dcm_tr(orientation_quat) *
                            WrapperWMM::instance().expected_mag(geo, day);
}

NumVector FullUKF::get_orientation_quaternion() const
{
    return x.segment(0, 4);
}

NumVector FullUKF::get_gyro_bias() const
{
    return x.segment(4, 3);
}

NumVector FullUKF::get_position() const
{
    return x.segment(7, 3);
}

NumVector FullUKF::get_velocity() const
{
    return x.segment(10, 3);
}

NumVector FullUKF::get_acceleration() const
{
    return x.segment(13, 3);
}

void FullUKF::set_proc_gyro_std(double std)
{
    params.proc_params.gyro_std = std;
}

void FullUKF::set_proc_gyro_bias_std(double std)
{
    params.proc_params.gyro_bias_std = std;
}

void FullUKF::set_proc_accel_std(double std)
{
    params.proc_params.accel_std = std;
}

void FullUKF::set_meas_accel_std(double std)
{
    params.meas_params.accel_std = std;
}

void FullUKF::set_meas_magn_std(double std)
{
    params.meas_params.magn_std = std;
}

void FullUKF::set_meas_pos_std(double std)
{
    params.meas_params.gps_cep = std;
}

void FullUKF::set_meas_vel_std(double std)
{
    params.meas_params.gps_vel_std = std;
}

void FullUKF::set_init_qs_std(double std)
{
    params.init_params.qs_std = std;
}

void FullUKF::set_init_qx_std(double std)
{
    params.init_params.qx_std = std;
}

void FullUKF::set_init_qy_std(double std)
{
    params.init_params.qy_std = std;
}

void FullUKF::set_init_qz_std(double std)
{
    params.init_params.qz_std = std;
}

void FullUKF::set_init_bias_std(double std)
{
    params.init_params.bias_std = std;
}

void FullUKF::set_init_pos_std(double std)
{
    params.init_params.pos_std = std;
}

void FullUKF::set_init_vel_std(double std)
{
    params.init_params.vel_std = std;
}

void FullUKF::set_init_accel_std(double std)
{
    params.init_params.accel_std = std;
}
