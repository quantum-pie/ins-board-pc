#include "fullekf.h"
#include "geometry.h"
#include "quatutils.h"

#include <Eigen/Dense>

using namespace quat;
using namespace geom;

FullEKF::FullEKF(const FilterParams & par)
    : is_initialized { false },
      params { par },
      bias_ctrl{ par.accum_capacity, Vector3D::Zero() }
{
    x = state_type::Zero();
    P = P_type::Identity();

    local_cov = create_local_cov_mtx();
}

FullEKF::~FullEKF() = default;

void FullEKF::step(const FilterInput & z)
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

void FullEKF::reset()
{
    is_initialized = false;
}

void FullEKF::initialize(const FilterInput & z)
{
    x.segment<4>(0) = static_cast<Quaternion::vector_form>(accel_magn_quat(z.a, z.m).conjugate());
    x.segment<3>(4) = bias_ctrl.get_mean();

    x.segment<3>(7) = z.pos;
    x.segment<3>(10) = z.v;
    x.segment<3>(13) = Vector3D::Zero();

    auto diag = P.diagonal();
    diag[0] = params.ori_init_params.qs_std * params.ori_init_params.qs_std;
    diag[1] = params.ori_init_params.qx_std * params.ori_init_params.qx_std;
    diag[2] = params.ori_init_params.qy_std * params.ori_init_params.qy_std;
    diag[3] = params.ori_init_params.qz_std * params.ori_init_params.qz_std;
    diag.segment<3>(4) = Vector3D::Constant(params.ori_init_params.bias_std * params.ori_init_params.bias_std);
    diag.segment<3>(7) = Vector3D::Constant(params.pos_init_params.pos_std * params.pos_init_params.pos_std);
    diag.segment<3>(10) = Vector3D::Constant(params.pos_init_params.vel_std * params.pos_init_params.vel_std);
    diag.segment<3>(13) = Vector3D::Constant(params.pos_init_params.accel_std * params.pos_init_params.accel_std);

    is_initialized = true;
    bias_ctrl.set_sampling(0); // free memory
}

void FullEKF::step_uninitialized(const FilterInput & z)
{
    bias_ctrl.update(z.w);
}

void FullEKF::step_initialized(const FilterInput & z)
{
    F_type F = create_transition_mtx(z);
    Q_type Q = create_proc_noise_cov_mtx(z.dt);

    x = F * x;
    normalize_state();

    P = F * P * F.transpose() + Q;

    if(z.gps_valid)
    {
        Vector3D geo = get_geodetic();

        Vector3D predicted_acc = predict_accelerometer(get_orientation_quaternion(), ecef_to_enu(get_acceleration(), geo), earth_model.gravity(geo));
        Vector3D predicted_magn = predict_magnetometer(get_orientation_quaternion(), earth_model.magnetic_vector(geo, z.day));

        meas_type z_pr;
        z_pr << predicted_acc, predicted_magn, get_cartesian(), get_velocity();

        meas_type z_meas;
        z_meas << z.a, z.m, z.pos, z.v;

        auto y = z_meas - z_pr;

        R_type R = create_meas_noise_cov_mtx(geo, z.day);
        H_type H = create_meas_proj_mtx(geo, z.day);

        auto S = H * P * H.transpose() + R;
        auto K = P * H.transpose() * S.inverse();

        x += K * y;
        normalize_state();

        auto tmp = P_type::Identity() - K * H;
        P = tmp * P * tmp.transpose() + K * R * K.transpose();
    }
}

void FullEKF::normalize_state()
{
    x.segment<4>(0) = static_cast<Quaternion::vector_form>(get_orientation_quaternion().normalize());
}

FullEKF::F_type FullEKF::create_transition_mtx(const FilterInput & z) const
{
    /* useful constants */
    double dt = z.dt;
    double dt_sq = z.dt * z.dt;
    double dt_2 = z.dt / 2;
    double dt_sq_2 = dt_sq / 2;

    /* constructing state transition matrix */
    auto V = skew_symmetric(z.w);

    V *= dt_2;
    V += StaticMatrix<4, 4>::Identity();

    auto K = get_orientation_quaternion().delta_mtx(dt_2);

    F_type F;
    const auto I3 = Matrix3D::Identity();

    F << V, K, StaticMatrix<4, 9>::Zero(),
           StaticMatrix<3, 4>::Zero(),  I3, StaticMatrix<3, 9>::Zero(),
           StaticMatrix<3, 7>::Zero(),  I3, dt * I3, dt_sq_2 * I3,
           StaticMatrix<3, 10>::Zero(), I3, dt * I3,
           StaticMatrix<3, 13>::Zero(), I3;

    return F;
}

FullEKF::Q_type FullEKF::create_proc_noise_cov_mtx(double dt) const
{
    /* useful constants */
    double dt_sq = dt * dt;
    double dt_2 = dt / 2;
    double dt_sq_2 = dt_sq / 2;

    auto K = get_orientation_quaternion().delta_mtx(dt_2);

    auto Qq = params.ori_proc_params.gyro_std * params.ori_proc_params.gyro_std * K * K.transpose();
    auto Qb = params.ori_proc_params.gyro_bias_std * params.ori_proc_params.gyro_bias_std * Matrix3D::Identity();

    StaticMatrix<9, 3> G;
    const auto I3 = Matrix3D::Identity();

    G << I3 * dt_sq_2,
         I3 * dt,
         I3;

    auto Qp = params.pos_proc_params.accel_std * params.pos_proc_params.accel_std * G * G.transpose();

    Q_type Q;
    Q << Qq, StaticMatrix<4, 12>::Zero(),
            StaticMatrix<3, 4>::Zero(), Qb, StaticMatrix<3, 9>::Zero(),
            StaticMatrix<9, 7>::Zero(), Qp;

    return Q;
}

FullEKF::R_type FullEKF::create_meas_noise_cov_mtx(const Vector3D & geo,
                                                   const boost::gregorian::date & day) const
{
    auto Ra = params.ori_meas_params.accel_std * params.ori_meas_params.accel_std * Matrix3D::Identity();

    double mag_magn = earth_model.magnetic_magnitude(geo, day);
    double normalized_magn_std = params.ori_meas_params.magn_std / mag_magn;
    auto Rm = normalized_magn_std * normalized_magn_std * Matrix3D::Identity();

    auto Cel = geodetic_to_dcm(geo);
    auto Rp = Cel.transpose() * local_cov * Cel;

    R_type R;

    double vel_variance = params.pos_meas_params.gps_vel_std * params.pos_meas_params.gps_vel_std;

    R << Ra, StaticMatrix<3, 9>::Zero(),
            Matrix3D::Zero(), Rm, StaticMatrix<3, 6>::Zero(),
            StaticMatrix<3, 6>::Zero(), Rp, Matrix3D::Zero(),
            StaticMatrix<3, 9>::Zero(), Matrix3D::Identity() * vel_variance;

    return R;
}

Matrix3D FullEKF::create_local_cov_mtx() const
{
    const double horizontal_linear_std = params.pos_meas_params.gps_cep * 1.2;
    const double altitude_std = horizontal_linear_std / 0.53;

    Matrix3D cov = Matrix3D::Zero();
    cov(0, 0) = horizontal_linear_std * horizontal_linear_std;
    cov(1, 1) = horizontal_linear_std * horizontal_linear_std;
    cov(2, 2) = altitude_std * altitude_std;

    return cov;
}

FullEKF::H_type FullEKF::create_meas_proj_mtx(const Vector3D & geo,
                                              const boost::gregorian::date & day) const
{
    // 1
    StaticMatrix<3, 4> Dac_Dq;

    double height_adjust = earth_model.gravity(geo) / Gravity::gf;

    Vector3D a = get_acceleration() / Gravity::gf;
    const Quaternion & q = get_orientation_quaternion();

    Matrix3D Cel = geodetic_to_dcm(geo);
    Matrix3D Clb = q.dcm_tr();
    Matrix3D Ceb = Clb * Cel;

    Matrix3D Ddcm_Dqs = q.ddcm_dqs_tr();
    Matrix3D Ddcm_Dqx = q.ddcm_dqx_tr();
    Matrix3D Ddcm_Dqy = q.ddcm_dqy_tr();
    Matrix3D Ddcm_Dqz = q.ddcm_dqz_tr();

    Vector3D tmp = Cel * a;

    Vector3D col_s = Ddcm_Dqs.col(2) * height_adjust + Ddcm_Dqs * tmp;
    Vector3D col_x = Ddcm_Dqx.col(2) * height_adjust + Ddcm_Dqx * tmp;
    Vector3D col_y = Ddcm_Dqy.col(2) * height_adjust + Ddcm_Dqy * tmp;
    Vector3D col_z = Ddcm_Dqz.col(2) * height_adjust + Ddcm_Dqz * tmp;

    Dac_Dq << col_s, col_x, col_y, col_z;

    // 2
    Matrix3D Dac_Dpos;

    Matrix3D Dgeo_Dpos = dgeo_dpos(geo, earth_model.get_ellipsoid());
    Matrix3D Ddcm_Dlat = dcm_lat_partial(geo);
    Matrix3D Ddcm_Dlon = dcm_lon_partial(geo);

    Matrix3D Dcel_Dx = Dgeo_Dpos(0, 0) * Ddcm_Dlat + Dgeo_Dpos(1, 0) * Ddcm_Dlon;
    Matrix3D Dcel_Dy = Dgeo_Dpos(0, 1) * Ddcm_Dlat + Dgeo_Dpos(1, 1) * Ddcm_Dlon;
    Matrix3D Dcel_Dz = Dgeo_Dpos(0, 2) * Ddcm_Dlat + Dgeo_Dpos(1, 2) * Ddcm_Dlon;

    col_x = Clb * Dcel_Dx * a;
    col_y = Clb * Dcel_Dy * a;
    col_z = Clb * Dcel_Dz * a;

    Dac_Dpos << col_x, col_y, col_z;

    // 3
    Matrix3D Dac_Da = Ceb / Gravity::gf;

    // 4
    StaticMatrix<3, 4> Dm_Dq;

    Vector3D earth_mag = earth_model.magnetic_vector(geo, day);

    col_s = Ddcm_Dqs * earth_mag;
    col_x = Ddcm_Dqx * earth_mag;
    col_y = Ddcm_Dqy * earth_mag;
    col_z = Ddcm_Dqz * earth_mag;

    Dm_Dq << col_s, col_x, col_y, col_z;

    // 5
    Matrix3D Dpos_Dpos = Matrix3D::Identity();

    // 6
    Matrix3D Dv_Dv = Matrix3D::Identity();

    // Combine
    H_type H;
    H <<    Dac_Dq, Matrix3D::Zero(), Dac_Dpos, Matrix3D::Zero(), Dac_Da,
            Dm_Dq, StaticMatrix<3, 12>::Zero(),
            StaticMatrix<3, 7>::Zero(), Dpos_Dpos, StaticMatrix<3, 6>::Zero(),
            StaticMatrix<3, 10>::Zero(), Dv_Dv, Matrix3D::Zero();

    return H;
}

Vector3D FullEKF::get_cartesian() const
{
    return x.segment<3>(7);
}

Vector3D FullEKF::get_geodetic() const
{
    return cartesian_to_geodetic(get_cartesian(), earth_model.get_ellipsoid());
}

Vector3D FullEKF::get_velocity() const
{
    return x.segment<3>(10);
}

Vector3D FullEKF::get_acceleration() const
{
    return x.segment<3>(13);
}

Quaternion FullEKF::get_orientation_quaternion() const
{
    return static_cast<Quaternion::vector_form>(x.segment<4>(0));
}

Vector3D FullEKF::get_gyro_bias() const
{
    return x.segment<3>(4);
}

void FullEKF::set_proc_gyro_std(double std)
{
    params.ori_proc_params.gyro_std = std;
}

void FullEKF::set_proc_gyro_bias_std(double std)
{
    params.ori_proc_params.gyro_bias_std = std;
}

void FullEKF::set_proc_accel_std(double std)
{
    params.pos_proc_params.accel_std = std;
}

void FullEKF::set_meas_accel_std(double std)
{
    params.ori_meas_params.accel_std = std;
}

void FullEKF::set_meas_magn_std(double std)
{
    params.ori_meas_params.magn_std = std;
}

void FullEKF::set_meas_pos_std(double std)
{
    params.pos_meas_params.gps_cep = std;
}

void FullEKF::set_meas_vel_std(double std)
{
    params.pos_meas_params.gps_vel_std = std;
}

void FullEKF::set_init_qs_std(double std)
{
    params.ori_init_params.qs_std = std;
}

void FullEKF::set_init_qx_std(double std)
{
    params.ori_init_params.qx_std = std;
}

void FullEKF::set_init_qy_std(double std)
{
    params.ori_init_params.qy_std = std;
}

void FullEKF::set_init_qz_std(double std)
{
    params.ori_init_params.qz_std = std;
}

void FullEKF::set_init_bias_std(double std)
{
    params.ori_init_params.bias_std = std;
}

void FullEKF::set_init_pos_std(double std)
{
    params.pos_init_params.pos_std = std;
}

void FullEKF::set_init_vel_std(double std)
{
    params.pos_init_params.vel_std = std;
}

void FullEKF::set_init_accel_std(double std)
{
    params.pos_init_params.accel_std = std;
}

double FullEKF::get_proc_gyro_std() const
{
    return params.ori_proc_params.gyro_std;
}

double FullEKF::get_proc_gyro_bias_std() const
{
    return params.ori_proc_params.gyro_bias_std;
}

double FullEKF::get_proc_accel_std() const
{
    return params.pos_proc_params.accel_std;
}

double FullEKF::get_meas_accel_std() const
{
    return params.ori_meas_params.accel_std;
}

double FullEKF::get_meas_magn_std() const
{
    return params.ori_meas_params.magn_std;
}

double FullEKF::get_meas_pos_std() const
{
    return params.pos_meas_params.gps_cep;
}

double FullEKF::get_meas_vel_std() const
{
    return params.pos_meas_params.gps_vel_std;
}

double FullEKF::get_init_qs_std() const
{
    return params.ori_init_params.qs_std;
}

double FullEKF::get_init_qx_std() const
{
    return params.ori_init_params.qx_std;
}

double FullEKF::get_init_qy_std() const
{
    return params.ori_init_params.qy_std;
}

double FullEKF::get_init_qz_std() const
{
    return params.ori_init_params.qz_std;
}

double FullEKF::get_init_bias_std() const
{
    return params.ori_init_params.bias_std;
}

double FullEKF::get_init_pos_std() const
{
    return params.pos_init_params.pos_std;
}

double FullEKF::get_init_vel_std() const
{
    return params.pos_init_params.vel_std;
}

double FullEKF::get_init_accel_std() const
{
    return params.pos_init_params.accel_std;
}
