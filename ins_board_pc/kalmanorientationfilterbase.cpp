#include "kalmanorientationfilterbase.h"
#include "quatutils.h"
#include "gravity.h"
#include "packets.h"

using namespace quat;

KalmanOrientationFilterBase::KalmanOrientationFilterBase(const FilterParams & params)
    : params{params}
{}

KalmanOrientationFilterBase::~KalmanOrientationFilterBase() = default;

KalmanOrientationFilterBase::F_type
KalmanOrientationFilterBase::create_transition_mtx(const FilterInput & z) const
{
    /* useful constants */
    double dt_2 = z.dt / 2;

    /* constructing state transition matrix */
    auto V = skew_symmetric(z.w);

    V *= dt_2;
    V += V_type::Identity();

    auto K = get_orientation_quaternion().delta_mtx(dt_2);

    F_type F;;
    F << V, K,
         StaticMatrix<3, 4>::Zero(), Matrix3D::Identity();

    return F;
}

KalmanOrientationFilterBase::P_type
KalmanOrientationFilterBase::create_init_cov_mtx() const
{
    P_type P;

    auto diag = P.diagonal();
    diag[0] = params.init_params.qs_std * params.init_params.qs_std;
    diag[1] = params.init_params.qx_std * params.init_params.qx_std;
    diag[2] = params.init_params.qy_std * params.init_params.qy_std;
    diag[3] = params.init_params.qz_std * params.init_params.qz_std;
    diag.segment<3>(4) = Vector3D::Constant(params.init_params.bias_std * params.init_params.bias_std);

    return P;
}

KalmanOrientationFilterBase::Q_type
KalmanOrientationFilterBase::create_proc_noise_cov_mtx(double dt) const
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

KalmanOrientationFilterBase::R_type
KalmanOrientationFilterBase::create_meas_noise_cov_mtx(double mag_magn) const
{
    auto Ra = params.meas_params.accel_std * params.meas_params.accel_std * Matrix3D::Identity();

    double normalized_magn_std = params.meas_params.magn_std / mag_magn;
    auto Rm = normalized_magn_std * normalized_magn_std * Matrix3D::Identity();

    R_type R;
    R << Ra, Matrix3D::Zero(),
         Matrix3D::Zero(), Rm;

    return R;
}

KalmanOrientationFilterBase::H_type
KalmanOrientationFilterBase::create_meas_proj_mtx(const Vector3D & earth_mag,
                                                  double gravity) const
{
    // 1
    StaticMatrix<3, 4> Dac_Dq;

    double height_adjust = gravity / Gravity::gf;

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

void KalmanOrientationFilterBase::do_set_proc_gyro_std(double std)
{
    params.proc_params.gyro_std = std;
}

void KalmanOrientationFilterBase::do_set_proc_gyro_bias_std(double std)
{
    params.proc_params.gyro_bias_std = std;
}

void KalmanOrientationFilterBase::do_set_meas_accel_std(double std)
{
    params.meas_params.accel_std = std;
}

void KalmanOrientationFilterBase::do_set_meas_magn_std(double std)
{
    params.meas_params.magn_std = std;
}

void KalmanOrientationFilterBase::do_set_init_qs_std(double std)
{
    params.init_params.qs_std = std;
}

void KalmanOrientationFilterBase::do_set_init_qx_std(double std)
{
    params.init_params.qx_std = std;
}

void KalmanOrientationFilterBase::do_set_init_qy_std(double std)
{
    params.init_params.qy_std = std;
}

void KalmanOrientationFilterBase::do_set_init_qz_std(double std)
{
    params.init_params.qz_std = std;
}

void KalmanOrientationFilterBase::do_set_init_bias_std(double std)
{
    params.init_params.bias_std = std;
}

double KalmanOrientationFilterBase::do_get_proc_gyro_std() const
{
    return params.proc_params.gyro_std;
}

double KalmanOrientationFilterBase::do_get_proc_gyro_bias_std() const
{
    return params.proc_params.gyro_bias_std;
}

double KalmanOrientationFilterBase::do_get_meas_accel_std() const
{
    return params.meas_params.accel_std;
}

double KalmanOrientationFilterBase::do_get_meas_magn_std() const
{
    return params.meas_params.magn_std;
}

double KalmanOrientationFilterBase::do_get_init_qs_std() const
{
    return params.init_params.qs_std;
}

double KalmanOrientationFilterBase::do_get_init_qx_std() const
{
    return params.init_params.qx_std;
}

double KalmanOrientationFilterBase::do_get_init_qy_std() const
{
    return params.init_params.qy_std;
}

double KalmanOrientationFilterBase::do_get_init_qz_std() const
{
    return params.init_params.qz_std;
}

double KalmanOrientationFilterBase::do_get_init_bias_std() const
{
    return params.init_params.bias_std;
}
