#include "kalmanorientationfilterbase.h"
#include "quatutils.h"
#include "earth.h"
#include "geometry.h"
#include "packets.h"

const std::size_t KalmanOrientationFilterBase::accum_size { 500 }; //!< Size of filter input accumulator.

const KalmanOrientationFilterBase::ProcessNoiseParams 	  KalmanOrientationFilterBase::default_proc_noise_params { 0.001, 0 };
const KalmanOrientationFilterBase::MeasurementNoiseParams KalmanOrientationFilterBase::default_meas_noise_params { 0.005, 1.2 };
const KalmanOrientationFilterBase::InitCovParams          KalmanOrientationFilterBase::default_init_cov_params { 0.0001, 0.00001, 0.00001, 0.0001, 0 };

using namespace quat;

KalmanOrientationFilterBase::KalmanOrientationFilterBase(const Ellipsoid & el)
    : earth_model{ el },
      x{ state_type::Zero() },
      P{ P_type::Identity() },
      bias_ctrl{ accum_size },
      initialized{ false },
      params{ default_proc_noise_params, default_meas_noise_params, default_init_cov_params }
{}

KalmanOrientationFilterBase::meas_type
KalmanOrientationFilterBase::do_get_true_measurement(const FilterInput & z) const
{
    meas_type meas;
    meas << z.a, z.m;
    return meas;
}

KalmanOrientationFilterBase::meas_type
KalmanOrientationFilterBase::do_get_predicted_measurement(const Vector3D & geo, const boost::gregorian::date & day) const
{
    Vector3D predicted_acc = geom::predict_accelerometer(get_orientation_quaternion(), earth_model.gravity(geo));
    Vector3D predicted_magn = geom::predict_magnetometer(get_orientation_quaternion(), earth_model.magnetic_vector(geo, day));

    meas_type pred;
    pred << predicted_acc, predicted_magn;

    return pred;
}

KalmanOrientationFilterBase::state_type
KalmanOrientationFilterBase::do_get_state() const
{
    return x;
}

void KalmanOrientationFilterBase::do_set_state(const state_type & st)
{
    x = st;
    x.segment<4>(0) = static_cast<vector_form>(get_orientation_quaternion().normalize());
}

KalmanOrientationFilterBase::P_type
KalmanOrientationFilterBase::do_get_cov() const
{
    return P;
}

void KalmanOrientationFilterBase::do_set_cov(const P_type & cov)
{
    P = cov;
}

Vector3D KalmanOrientationFilterBase::do_get_geodetic(const FilterInput & z) const
{
    return z.geo;
}

bool KalmanOrientationFilterBase::do_is_initialized() const
{
    return initialized;
}

bool KalmanOrientationFilterBase::do_is_ready_to_initialize() const
{
    return bias_ctrl.is_saturated();
}

void KalmanOrientationFilterBase::do_initialize(const FilterInput & z)
{
    // TODO check and correct if wrong
    x.segment<4>(0) = static_cast<vector_form>( geom::accel_magn_quat(z.a, z.m, earth_model.magnetic_declination(z.geo, z.day)).conjugate() );
    x.segment<3>(4) = bias_ctrl.get_mean();

    P = create_init_cov_mtx();

    initialized = true;
    bias_ctrl.set_sampling(0); // free memory
}

void KalmanOrientationFilterBase::do_accumulate(const FilterInput & z)
{
    bias_ctrl.update(z.w);
}

KalmanOrientationFilterBase::F_type
KalmanOrientationFilterBase::do_create_transition_mtx(const FilterInput & z) const
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
KalmanOrientationFilterBase::do_create_init_cov_mtx() const
{
    P_type P = P_type::Zero();

    auto diag = P.diagonal();
    diag[0] = params.init_params.qs_std * params.init_params.qs_std;
    diag[1] = params.init_params.qx_std * params.init_params.qx_std;
    diag[2] = params.init_params.qy_std * params.init_params.qy_std;
    diag[3] = params.init_params.qz_std * params.init_params.qz_std;
    diag.segment<3>(4) = Vector3D::Constant(params.init_params.bias_std * params.init_params.bias_std);

    return P;
}

KalmanOrientationFilterBase::Q_type
KalmanOrientationFilterBase::do_create_proc_noise_cov_mtx(double dt) const
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
KalmanOrientationFilterBase::do_create_meas_noise_cov_mtx(const Vector3D & geo, const boost::gregorian::date & day) const
{
    auto Ra = params.meas_params.accel_std * params.meas_params.accel_std * Matrix3D::Identity();

    double normalized_magn_std = params.meas_params.magn_std / earth_model.magnetic_magnitude(geo, day);
    auto Rm = normalized_magn_std * normalized_magn_std * Matrix3D::Identity();

    R_type R;
    R << Ra, Matrix3D::Zero(),
         Matrix3D::Zero(), Rm;

    return R;
}

KalmanOrientationFilterBase::H_type
KalmanOrientationFilterBase::do_create_meas_proj_mtx(const Vector3D & geo, const boost::gregorian::date & day) const
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

void KalmanOrientationFilterBase::do_reset()
{
    initialized = false;
    bias_ctrl.set_sampling(accum_size);
}

Quaternion KalmanOrientationFilterBase::do_get_orientation_quaternion() const
{
    return static_cast<vector_form>(x.segment<4>(0));
}

Vector3D KalmanOrientationFilterBase::do_get_gyro_bias() const
{
    return x.segment<3>(4);
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
