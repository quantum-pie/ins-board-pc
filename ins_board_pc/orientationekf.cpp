#include "orientationekf.h"
#include "geometry.h"
#include "quatutils.h"

#include <Eigen/Dense>

using namespace quat;
using namespace geom;

OrientationEKF::OrientationEKF(const FilterParams & par)
    : is_initialized{ false },
      params{ par },
      bias_ctrl{ par.accum_capacity, Vector3D::Zero() }
{
    x = state_type::Zero();
    P = P_type::Identity();
}

OrientationEKF::~OrientationEKF() = default;

void OrientationEKF::step(const FilterInput & z)
{
    if(is_initialized)
    {
        step_initialized(z);
    }
    else if(bias_ctrl.is_saturated())
    {
        initialize(z);
    }
    else
    {
        step_uninitialized(z);
    }
}

void OrientationEKF::reset()
{
    is_initialized = false;
}

void OrientationEKF::initialize(const FilterInput & z)
{
    // TODO check and correct in full EKF and UKF also if wrong
    x.segment<4>(0) = static_cast<Quaternion::vector_form>( accel_magn_quat(z.a, z.m, earth_model.magnetic_declination(z.geo, z.day)).conjugate() );
    x.segment<3>(4) = bias_ctrl.get_mean();

    auto diag = P.diagonal();
    diag[0] = params.init_params.qs_std * params.init_params.qs_std;
    diag[1] = params.init_params.qx_std * params.init_params.qx_std;
    diag[2] = params.init_params.qy_std * params.init_params.qy_std;
    diag[3] = params.init_params.qz_std * params.init_params.qz_std;
    diag.segment<3>(4) = Vector3D::Constant(params.init_params.bias_std * params.init_params.bias_std);

    is_initialized = true;
    bias_ctrl.set_sampling(0); // free memory
}

void OrientationEKF::step_uninitialized(const FilterInput & z)
{
    bias_ctrl.update(z.w);
}

void OrientationEKF::step_initialized(const FilterInput & z)
{
    F_type F = create_transition_mtx(z);
    Q_type Q = create_proc_noise_cov_mtx(z.dt);

    x = F * x;
    normalize_state();

    P = F * P * F.transpose() + Q;

    if(z.gps_valid)
    {
        Vector3D predicted_acc = predict_accelerometer(get_orientation_quaternion(), Vector3D::Zero(), earth_model.gravity(z.geo));
        Vector3D predicted_magn = predict_magnetometer(get_orientation_quaternion(), earth_model.magnetic_vector(z.geo, z.day));

        meas_type z_pr;
        z_pr << predicted_acc, predicted_magn;

        meas_type z_meas;
        z_meas << z.a, z.m;

        auto y = z_meas - z_pr;

        R_type R = create_meas_noise_cov_mtx(z.geo, z.day);
        H_type H = create_meas_proj_mtx(z.geo, z.day);

        auto S = H * P * H.transpose() + R;
        auto K = P * H.transpose() * S.inverse();

        x += K * y;
        normalize_state();

        auto tmp = P_type::Identity() - K * H;
        P = tmp * P * tmp.transpose() + K * R * K.transpose();
    }
}

void OrientationEKF::normalize_state()
{
    x.segment<4>(0) = static_cast<Quaternion::vector_form>(get_orientation_quaternion().normalize());
}

OrientationEKF::F_type OrientationEKF::create_transition_mtx(const FilterInput & z) const
{
    /* useful constants */
    double dt_2 = z.dt / 2;

    /* constructing state transition matrix */
    auto V = skew_symmetric(z.w);

    V *= dt_2;
    V += StaticMatrix<4, 4>::Identity();

    auto K = get_orientation_quaternion().delta_mtx(dt_2);

    F_type F;;
    F << V, K,
         StaticMatrix<3, 4>::Zero(), Matrix3D::Identity();

    return F;
}

OrientationEKF::Q_type OrientationEKF::create_proc_noise_cov_mtx(double dt) const
{
    /* useful constants */
    double dt_2 = dt / 2;

    auto K = get_orientation_quaternion().delta_mtx(dt_2);

    auto Qq = params.proc_params.gyro_std * params.proc_params.gyro_std * K * K.transpose();
    auto Qb = params.proc_params.gyro_bias_std * params.proc_params.gyro_bias_std * Matrix3D::Identity();

    Q_type Q;
    Q << Qq, StaticMatrix<4, 3>::Zero(),
            StaticMatrix<3, 4>::Zero(), Qb;

    return Q;
}

OrientationEKF::R_type OrientationEKF::create_meas_noise_cov_mtx(const Vector3D & geo,
                                                   const boost::gregorian::date & day) const
{
    auto Ra = params.meas_params.accel_std * params.meas_params.accel_std * Matrix3D::Identity();

    double mag_magn = earth_model.magnetic_magnitude(geo, day);
    double normalized_magn_std = params.meas_params.magn_std / mag_magn;
    auto Rm = normalized_magn_std * normalized_magn_std * Matrix3D::Identity();

    R_type R;
    R << Ra, Matrix3D::Zero(),
         Matrix3D::Zero(), Rm;

    return R;
}

OrientationEKF::H_type OrientationEKF::create_meas_proj_mtx(const Vector3D & geo,
                                              const boost::gregorian::date & day) const
{
    // 1
    StaticMatrix<3, 4> Dac_Dq;

    double height_adjust = earth_model.gravity(geo) / Gravity::gf;

    const Quaternion & q = get_orientation_quaternion();

    Matrix3D Ddcm_Dqs = q.ddcm_dqs_tr();
    Matrix3D Ddcm_Dqx = q.ddcm_dqx_tr();
    Matrix3D Ddcm_Dqy = q.ddcm_dqy_tr();
    Matrix3D Ddcm_Dqz = q.ddcm_dqz_tr();

    Vector3D col_s = Ddcm_Dqs.col(2) * height_adjust;
    Vector3D col_x = Ddcm_Dqx.col(2) * height_adjust;
    Vector3D col_y = Ddcm_Dqy.col(2) * height_adjust;
    Vector3D col_z = Ddcm_Dqz.col(2) * height_adjust;

    Dac_Dq << col_s, col_x, col_y, col_z;

    // 2
    StaticMatrix<3, 4> Dm_Dq;

    Vector3D earth_mag = earth_model.magnetic_vector(geo, day);

    col_s = Ddcm_Dqs * earth_mag;
    col_x = Ddcm_Dqx * earth_mag;
    col_y = Ddcm_Dqy * earth_mag;
    col_z = Ddcm_Dqz * earth_mag;

    Dm_Dq << col_s, col_x, col_y, col_z;

    // Combine
    H_type H;
    H <<    Dac_Dq, Matrix3D::Zero(),
            Dm_Dq, Matrix3D::Zero();

    return H;
}

Quaternion OrientationEKF::get_orientation_quaternion() const
{
    return static_cast<Quaternion::vector_form>(x.segment<4>(0));
}

Vector3D OrientationEKF::get_gyro_bias() const
{
    return x.segment<3>(4);
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

double OrientationEKF::get_proc_gyro_std() const
{
    return params.proc_params.gyro_std;
}

double OrientationEKF::get_proc_gyro_bias_std() const
{
    return params.proc_params.gyro_bias_std;
}

double OrientationEKF::get_meas_accel_std() const
{
    return params.meas_params.accel_std;
}

double OrientationEKF::get_meas_magn_std() const
{
    return params.meas_params.magn_std;
}

double OrientationEKF::get_init_qs_std() const
{
    return params.init_params.qs_std;
}

double OrientationEKF::get_init_qx_std() const
{
    return params.init_params.qx_std;
}

double OrientationEKF::get_init_qy_std() const
{
    return params.init_params.qy_std;
}

double OrientationEKF::get_init_qz_std() const
{
    return params.init_params.qz_std;
}

double OrientationEKF::get_init_bias_std() const
{
    return params.init_params.bias_std;
}
