#include "fullukf.h"
#include "geometry.h"

#include <Eigen/Dense>

#include <cmath>

FullUKF::FullUKF(const FilterParams & par)
    : is_initialized { false },
      params { par }
{
    x = state_type::Zero();
    P = P_type::Identity();

    local_cov = create_local_cov_mtx();

    double alpha_sq = params.ut_params.alpha * params.ut_params.alpha;
    lambda = alpha_sq * (L + params.ut_params.kappa) - L;

    Ws[0] = lambda / (L + lambda);
    Wc[0] = Ws[0] + (1 - alpha_sq + params.ut_params.beta);

    for(int i = 1; i <= 2 * L; ++i)
    {
        Ws[i] = 0.5 / (L + lambda);
        Wc[i] = 0.5 / (L + lambda);
    }
}

FullUKF::~FullUKF() = default;

void FullUKF::step(const FilterInput & z)
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

void FullUKF::reset()
{
    is_initialized = false;
}

void FullUKF::initialize(const FilterInput & z)
{
    x.segment<4>(0) = static_cast<Quaternion::vector_form>(Quaternion::accel_magn_quat(z.a, z.m).conjugate());
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

void FullUKF::step_uninitialized(const FilterInput & z)
{
    bias_ctrl.update(z.w);
}

void FullUKF::step_initialized(const FilterInput & z)
{
    /* predict (EKF style) */
    F_type F = create_transition_mtx(z);
    Q_type Q = create_proc_noise_cov_mtx(z.dt);

    x = F * x;
    normalize_state();

    P = F * P * F.transpose() + Q;

    if(z.gps_valid)
    {
        // update (unscented version)
        sigma_p[0] = x;

        P_type mtx_root = P.llt().matrixL();
        mtx_root *= std::sqrt(L + lambda);

        for(int i = 1; i <= L; ++i)
        {
            sigma_p[i] = x + mtx_root.col(i - 1);
            sigma_p[i + L] = x - mtx_root.col(i - 1);
        }

        meas_type z_p = meas_type::Zero();
        for(int i = 0; i <= 2 * L; ++i)
        {
            x = sigma_p[i];

            const auto & sigma_quat = get_orientation_quaternion();
            const auto & sigma_accel = get_acceleration();
            const auto & sigma_pos = get_cartesian();
            const auto & sigma_vel = get_velocity();

            auto geo = get_geodetic();
            auto pred_acc = calculate_accelerometer(sigma_quat, sigma_accel, geo);
            auto pred_magn = calculate_magnetometer(sigma_quat, geo, z.day);

            meas_type z_pr;
            z_pr << pred_acc, pred_magn, sigma_pos, sigma_vel;

            sigma_z[i] = z_pr;
            z_p += Ws[i] * z_pr;
        }

        x = sigma_p[0];

        R_type Pzz = R_type::Zero();
        K_type Pxz = K_type::Zero();

        for(int i = 0; i <= 2 * L; ++i)
        {
            Pzz += Wc[i] * (sigma_z[i] - z_p) * (sigma_z[i] - z_p).transpose();
            Pxz += Wc[i] * (sigma_p[i] - x ) * (sigma_z[i] - z_p).transpose();
        }

        R_type R = create_meas_noise_cov_mtx(z.geo, z.day);
        Pzz += R;

        K_type K = Pxz * Pzz.inverse();

        meas_type z_meas;
        z_meas << z.a, z.m, z.pos, z.v;

        x += K * (z_meas - z_p);
        normalize_state();

        P -= K * Pzz * K.transpose();
    }
}

void FullUKF::normalize_state()
{
    x.segment<4>(0) = static_cast<Quaternion::vector_form>(get_orientation_quaternion().normalize());
}

FullUKF::F_type FullUKF::create_transition_mtx(const FilterInput & z) const
{
    /* useful constants */
    double dt = z.dt;
    double dt_sq = z.dt * z.dt;
    double dt_2 = z.dt / 2;
    double dt_sq_2 = dt_sq / 2;

    /* constructing state transition matrix */
    auto V = Quaternion::skew_symmetric(z.w);

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

FullUKF::Q_type FullUKF::create_proc_noise_cov_mtx(double dt) const
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

FullUKF::R_type FullUKF::create_meas_noise_cov_mtx(const Vector3D & geo,
                                                   const boost::gregorian::date & day) const
{
    auto Ra = params.ori_meas_params.accel_std * params.ori_meas_params.accel_std * Matrix3D::Identity();

    double mag_magn = earth_model.magnetic_magnitude(geo, day);
    double normalized_magn_std = params.ori_meas_params.magn_std / mag_magn;
    auto Rm = normalized_magn_std * normalized_magn_std * Matrix3D::Identity();

    auto Cel = geom::geodetic_to_dcm(geo);
    auto Rp = Cel.transpose() * local_cov * Cel;

    R_type R;

    double vel_variance = params.pos_meas_params.gps_vel_std * params.pos_meas_params.gps_vel_std;

    R << Ra, StaticMatrix<3, 9>::Zero(),
            Matrix3D::Zero(), Rm, StaticMatrix<3, 6>::Zero(),
            StaticMatrix<3, 6>::Zero(), Rp, Matrix3D::Zero(),
            StaticMatrix<3, 9>::Zero(), Matrix3D::Identity() * vel_variance;

    return R;
}

Matrix3D FullUKF::create_local_cov_mtx() const
{
    const double horizontal_linear_std = params.pos_meas_params.gps_cep * 1.2;
    const double altitude_std = horizontal_linear_std / 0.53;

    Matrix3D cov = Matrix3D::Zero();
    cov(0, 0) = horizontal_linear_std * horizontal_linear_std;
    cov(1, 1) = horizontal_linear_std * horizontal_linear_std;
    cov(2, 2) = altitude_std * altitude_std;

    return cov;
}

Vector3D FullUKF::calculate_accelerometer(const Quaternion & orientation_quat, const Vector3D & acceleration,
                                          const Vector3D & geo) const
{
    double height_adjust = earth_model.gravity(geo) / Gravity::gf;

    Matrix3D Clb = orientation_quat.dcm_tr();
    Matrix3D Cel = geom::geodetic_to_dcm(geo);

    Vector3D movement_component = Clb * Cel * acceleration / Gravity::gf;

    Vector3D g;
    g << 0, 0, height_adjust;

    Vector3D gravity_component = Clb * g;

    return gravity_component + movement_component;
}

Vector3D FullUKF::calculate_magnetometer(const Quaternion & orientation_quat,
                                         const Vector3D & geo, const boost::gregorian::date & day) const
{
    return orientation_quat.dcm_tr() * earth_model.magnetic_vector(geo, day);
}

Vector3D FullUKF::get_cartesian() const
{
    return x.segment<3>(7);
}

Vector3D FullUKF::get_geodetic() const
{
    return geom::cartesian_to_geodetic(get_cartesian(), earth_model.get_ellipsoid());
}

Vector3D FullUKF::get_velocity() const
{
    return x.segment<3>(10);
}

Vector3D FullUKF::get_acceleration() const
{
    return x.segment<3>(13);
}

Quaternion FullUKF::get_orientation_quaternion() const
{
    return static_cast<Quaternion::vector_form>(x.segment<4>(0));
}

Vector3D FullUKF::get_gyro_bias() const
{
    return x.segment<3>(4);
}

void FullUKF::set_proc_gyro_std(double std)
{
    params.ori_proc_params.gyro_std = std;
}

void FullUKF::set_proc_gyro_bias_std(double std)
{
    params.ori_proc_params.gyro_bias_std = std;
}

void FullUKF::set_proc_accel_std(double std)
{
    params.pos_proc_params.accel_std = std;
}

void FullUKF::set_meas_accel_std(double std)
{
    params.ori_meas_params.accel_std = std;
}

void FullUKF::set_meas_magn_std(double std)
{
    params.ori_meas_params.magn_std = std;
}

void FullUKF::set_meas_pos_std(double std)
{
    params.pos_meas_params.gps_cep = std;
}

void FullUKF::set_meas_vel_std(double std)
{
    params.pos_meas_params.gps_vel_std = std;
}

void FullUKF::set_init_qs_std(double std)
{
    params.ori_init_params.qs_std = std;
}

void FullUKF::set_init_qx_std(double std)
{
    params.ori_init_params.qx_std = std;
}

void FullUKF::set_init_qy_std(double std)
{
    params.ori_init_params.qy_std = std;
}

void FullUKF::set_init_qz_std(double std)
{
    params.ori_init_params.qz_std = std;
}

void FullUKF::set_init_bias_std(double std)
{
    params.ori_init_params.bias_std = std;
}

void FullUKF::set_init_pos_std(double std)
{
    params.pos_init_params.pos_std = std;
}

void FullUKF::set_init_vel_std(double std)
{
    params.pos_init_params.vel_std = std;
}

void FullUKF::set_init_accel_std(double std)
{
    params.pos_init_params.accel_std = std;
}

double FullUKF::get_proc_gyro_std() const
{
    return params.ori_proc_params.gyro_std;
}

double FullUKF::get_proc_gyro_bias_std() const
{
    return params.ori_proc_params.gyro_bias_std;
}

double FullUKF::get_proc_accel_std() const
{
    return params.pos_proc_params.accel_std;
}

double FullUKF::get_meas_accel_std() const
{
    return params.ori_meas_params.accel_std;
}

double FullUKF::get_meas_magn_std() const
{
    return params.ori_meas_params.magn_std;
}

double FullUKF::get_meas_pos_std() const
{
    return params.pos_meas_params.gps_cep;
}

double FullUKF::get_meas_vel_std() const
{
    return params.pos_meas_params.gps_vel_std;
}

double FullUKF::get_init_qs_std() const
{
    return params.ori_init_params.qs_std;
}

double FullUKF::get_init_qx_std() const
{
    return params.ori_init_params.qx_std;
}

double FullUKF::get_init_qy_std() const
{
    return params.ori_init_params.qy_std;
}

double FullUKF::get_init_qz_std() const
{
    return params.ori_init_params.qz_std;
}

double FullUKF::get_init_bias_std() const
{
    return params.ori_init_params.bias_std;
}

double FullUKF::get_init_pos_std() const
{
    return params.pos_init_params.pos_std;
}

double FullUKF::get_init_vel_std() const
{
    return params.pos_init_params.vel_std;
}

double FullUKF::get_init_accel_std() const
{
    return params.pos_init_params.accel_std;
}
