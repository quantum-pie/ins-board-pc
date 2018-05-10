#include "kalmanpositionfilterbase.h"
#include "geometry.h"

KalmanPositionFilterBase::KalmanPositionFilterBase(const FilterParams & params)
    : params{params}
{}

KalmanPositionFilterBase::~KalmanPositionFilterBase() = default;


KalmanPositionFilterBase::F_type
KalmanPositionFilterBase::create_transition_mtx(double dt) const
{
    /* useful constants */
    const double dt_sq = dt * dt;
    const double dt_sq_2 = dt_sq / 2;

    /* constructing state transition matrix */
    F_type F;

    const auto I3 = Matrix3D::Identity();

    F << I3, dt * I3, dt_sq_2 * I3,
         StaticMatrix<3, 3>::Zero(), I3, dt * I3,
         StaticMatrix<3, 6>::Zero(), I3;

    return F;
}

KalmanPositionFilterBase::P_type
KalmanPositionFilterBase::create_init_cov_mtx() const
{
    P_type P;
    auto diag = P.diagonal();
    diag.segment<3>(0) = Vector3D::Constant(params.init_params.pos_std * params.init_params.pos_std);
    diag.segment<3>(3) = Vector3D::Constant(params.init_params.vel_std * params.init_params.vel_std);
    diag.segment<3>(6) = Vector3D::Constant(params.init_params.accel_std * params.init_params.accel_std);

    return P;
}

KalmanPositionFilterBase::Q_type
KalmanPositionFilterBase::create_proc_noise_cov_mtx(double dt) const
{
    /* useful constants */
    double dt_sq = dt * dt;
    double dt_sq_2 = dt_sq / 2;

    StaticMatrix<9, 3> G;

    const auto I3 = Matrix3D::Identity();

    G << I3 * dt_sq_2,
         I3 * dt,
         I3;

    return params.proc_params.accel_std * params.proc_params.accel_std * G * G.transpose();
}

KalmanPositionFilterBase::R_type
KalmanPositionFilterBase::create_meas_noise_cov_mtx(const Vector3D & geo) const
{
    double horizontal_linear_std = params.meas_params.gps_cep * 1.2;
    double altitude_std = horizontal_linear_std / 0.53;

    Matrix3D local_cov = Matrix3D::Zero();
    local_cov(0, 0) = horizontal_linear_std * horizontal_linear_std;
    local_cov(1, 1) = horizontal_linear_std * horizontal_linear_std;
    local_cov(2, 2) = altitude_std * altitude_std;

    Matrix3D Cel = geom::geodetic_to_dcm(geo);
    Matrix3D Rp = Cel.transpose() * local_cov * Cel;

    R_type R;

    double vel_variance = params.meas_params.gps_vel_std * params.meas_params.gps_vel_std;

    R << Rp, Matrix3D::Zero(),
         Matrix3D::Zero(), Matrix3D::Identity() * vel_variance;

    return R;
}

KalmanPositionFilterBase::H_type
KalmanPositionFilterBase::create_meas_proj_mtx() const
{
    H_type H;

    H <<   Matrix3D::Identity(), StaticMatrix<3, 6>::Zero(),
            Matrix3D::Zero(), Matrix3D::Identity(), Matrix3D::Zero();

    return H;
}

void KalmanPositionFilterBase::do_set_proc_accel_std(double std)
{
    params.proc_params.accel_std = std;
}

void KalmanPositionFilterBase::do_set_meas_pos_std(double std)
{
    params.meas_params.gps_cep = std;
}

void KalmanPositionFilterBase::do_set_meas_vel_std(double std)
{
    params.meas_params.gps_vel_std = std;
}

void KalmanPositionFilterBase::do_set_init_pos_std(double std)
{
    params.init_params.pos_std = std;
}

void KalmanPositionFilterBase::do_set_init_vel_std(double std)
{
    params.init_params.vel_std = std;
}

void KalmanPositionFilterBase::do_set_init_accel_std(double std)
{
    params.init_params.accel_std = std;
}

double KalmanPositionFilterBase::do_get_proc_accel_std() const
{
    return params.proc_params.accel_std;
}

double KalmanPositionFilterBase::do_get_meas_pos_std() const
{
    return params.meas_params.gps_cep;
}

double KalmanPositionFilterBase::do_get_meas_vel_std() const
{
    return params.meas_params.gps_vel_std;
}

double KalmanPositionFilterBase::do_get_init_pos_std() const
{
    return params.init_params.pos_std;
}

double KalmanPositionFilterBase::do_get_init_vel_std() const
{
    return params.init_params.vel_std;
}

double KalmanPositionFilterBase::do_get_init_accel_std() const
{
    return params.init_params.accel_std;
}
