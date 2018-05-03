#include "posukf.h"
#include "physconst.h"

#include <Eigen/Dense>

#include <QtMath>

const int PositionUnscented::state_size = 9;
const int PositionUnscented::measurement_size = 4;

PositionUnscented::PositionUnscented(const FilterParams & par)
    : AbstractKalmanPositionFilter(),
      params(par)
{
    x = NumVector(state_size);
    P = NumMatrix(state_size, state_size);
}

PositionUnscented::~PositionUnscented()
{

}

void PositionUnscented::initialize(const FilterInput & z)
{
    AbstractKalmanPositionFilter::initialize(z);

    x[0] = z.pos[0];
    x[1] = z.pos[1];
    x[2] = z.pos[2];

    x[3] = z.v[0];
    x[4] = z.v[1];
    x[5] = z.v[2];

    x[6] = 0;
    x[7] = 0;
    x[8] = 0;

    P = NumMatrix::Zero(state_size, state_size);

    double position_variance = params.init_params.pos_std * params.init_params.pos_std;
    P(0, 0) = position_variance;
    P(1, 1) = position_variance;
    P(2, 2) = position_variance;

    double velocity_variance = params.init_params.vel_std * params.init_params.vel_std;
    P(3, 3) = velocity_variance;
    P(4, 4) = velocity_variance;
    P(5, 5) = velocity_variance;

    double accel_variance = params.init_params.accel_std * params.init_params.accel_std;
    P(6, 6) = accel_variance;
    P(7, 7) = accel_variance;
    P(8, 8) = accel_variance;
}

void PositionUnscented::accumulate(const FilterInput &)
{

}

void PositionUnscented::step(const FilterInput & z)
{
    if(is_initialized())
    {
        update(z);
    }
    else
    {
        initialize(z);
    }
}

void PositionUnscented::update(const FilterInput & z)
{
    // predict (EKF version)
    NumMatrix F = create_transition_mtx(z);
    NumMatrix Q = create_proc_noise_cov_mtx(z.dt);

    x = F * x;
    P = F * P * F.transpose() + Q;

    //eaux::debug_matrix(P, "P");

    // update (unscented version)
    double L = state_size;
    double kappa = 0;
    double beta = 2;
    double alpha = 1e-3;
    double lambda = alpha * alpha * (L + kappa) - L;

    std::vector<NumVector> sigma_p(2 * L + 1);
    sigma_p[0] = x;

    NumMatrix mtx_root = P.llt().matrixL();
    mtx_root *= qSqrt(L + lambda);

    //eaux::debug_matrix(mtx_root, "mtx_root");
    //eaux::debug_matrix((aug_size + lambda) * P_aug, "mtx");
    //eaux::debug_matrix(mtx_root * mtx_root, "mtx_r2");

    for(int i = 1; i <= L; ++i)
    {
        sigma_p[i] = x + mtx_root.col(i - 1);
        sigma_p[i + L] = x - mtx_root.col(i - 1);

        //eaux::debug_vector(sigma_p[i], "sp1");
        //eaux::debug_vector(sigma_p[i + aug_size], "sp2");
    }

    std::vector<double> Ws(2 * L + 1);
    std::vector<double> Wc(2 * L + 1);

    Ws[0] = lambda / (L + lambda);
    Wc[0] = Ws[0] + 1 - alpha * alpha + beta;

    for(int i = 1; i <= 2 * L; ++i)
    {
        Ws[i] = 0.5 / (L + lambda);
        Wc[i] = 0.5 / (L + lambda);
    }

    std::vector<NumVector> sigma_z(2 * L + 1);
    NumVector z_p = NumMatrix::Zero(measurement_size, 1);

    for(int i = 0; i <= 2 * L; ++i)
    {
        x = sigma_p[i];

        NumVector sigma_pos = get_position();
        NumVector sigma_vel = get_velocity();

        //eaux::debug_vector(sigma_quat, "quat");
        //eaux::debug_vector(sigma_accel, "sigma_acc");
        //eaux::debug_vector(sigma_pos, "pos");
        //eaux::debug_vector(sigma_vel, "vel");

        double lat, lon, alt;
        calculate_geodetic(sigma_pos, lat, lon, alt);

        NumVector z_pr(measurement_size);

        //eaux::debug_vector(sigma_accel, "accel");

        //eaux::debug_vector(z_pr.segment(0, 3), "accel_calc");

        z_pr(0) = sigma_pos(0);
        z_pr(1) = sigma_pos(1);
        z_pr(2) = sigma_pos(2);

        calculate_velocity(sigma_vel, z_pr(3));

        sigma_z[i] = z_pr;
        z_p += Ws[i] * z_pr;
    }

    x = sigma_p[0];

    //eaux::debug_vector(z_p, "z_pred");

    NumMatrix Pzz = NumMatrix::Zero(measurement_size, measurement_size);
    NumMatrix Pxz = NumMatrix::Zero(state_size, measurement_size);

    for(int i = 0; i <= 2 * L; ++i)
    {
        Pzz += Wc[i] * (sigma_z[i] - z_p) * (sigma_z[i] - z_p).transpose();
        Pxz += Wc[i] * (sigma_p[i] - x ) * (sigma_z[i] - z_p).transpose();
    }

    NumMatrix R = create_meas_noise_cov_mtx(z.geo[0], z.geo[1]);
    Pzz += R;

    NumMatrix K = Pxz * Pzz.inverse();

    //eaux::debug_matrix(K, "K");

    NumVector z_meas(measurement_size);
    z_meas << z.pos, z.v.norm();

    //eaux::debug_vector(z_meas, "meas");
    //eaux::debug_vector(z_p, "predict");

    x += K * (z_meas - z_p);
    P -= K * Pzz * K.transpose();

    //eaux::debug_vector(get_position() - start_ecef, "delta");
    //eaux::debug_vector(x, "state");
}

NumMatrix PositionUnscented::create_transition_mtx(const FilterInput & z) const
{
    /* useful constants */
    double dt = z.dt;
    double dt_sq = z.dt * z.dt;
    double dt_sq_2 = dt_sq / 2;

    /* constructing state transition matrix */
    NumMatrix F(state_size, state_size);

    F << NumMatrix::Identity(3, 3), dt * NumMatrix::Identity(3, 3), dt_sq_2 * NumMatrix::Identity(3, 3),
            NumMatrix::Zero(3, 3), NumMatrix::Identity(3, 3), dt * NumMatrix::Identity(3, 3),
            NumMatrix::Zero(3, 6), NumMatrix::Identity(3, 3);

    return F;
}

NumMatrix PositionUnscented::create_proc_noise_cov_mtx(double dt) const
{
    /* useful constants */
    double dt_sq = dt * dt;
    double dt_sq_2 = dt_sq / 2;

    NumMatrix G(9, 3);
    G << NumMatrix::Identity(3, 3) * dt_sq_2,
            NumMatrix::Identity(3, 3) * dt,
            NumMatrix::Identity(3, 3);

    return params.proc_params.accel_std * params.proc_params.accel_std * G * G.transpose();
}

NumMatrix PositionUnscented::create_meas_noise_cov_mtx(double lat, double lon) const
{
    double horizontal_linear_std = params.meas_params.gps_cep * 1.2;
    double altitude_std = horizontal_linear_std / 0.53;

    NumMatrix Cel = WrapperWMM::instance().geodetic_to_dcm(lat, lon);
    NumMatrix local_cov = NumMatrix::Identity(3, 3);
    local_cov(0, 0) = horizontal_linear_std * horizontal_linear_std;
    local_cov(1, 1) = horizontal_linear_std * horizontal_linear_std;
    local_cov(2, 2) = altitude_std * altitude_std;

    NumMatrix Rp = Cel.transpose() * local_cov * Cel;

    NumMatrix R(measurement_size, measurement_size);

    R << Rp, NumMatrix::Zero(3, 1),
            NumMatrix::Zero(1, 3), params.meas_params.gps_vel_abs_std * params.meas_params.gps_vel_abs_std;

    return R;
}

NumVector PositionUnscented::get_position() const
{
    return x.segment(0, 3);
}

NumVector PositionUnscented::get_velocity() const
{
    return x.segment(3, 3);
}

NumVector PositionUnscented::get_acceleration() const
{
    return x.segment(6, 3);
}

void PositionUnscented::set_proc_accel_std(double std)
{
    params.proc_params.accel_std = std;
}

void PositionUnscented::set_meas_pos_std(double std)
{
    params.meas_params.gps_cep = std;
}

void PositionUnscented::set_meas_vel_std(double std)
{
    params.meas_params.gps_vel_abs_std = std;
}

void PositionUnscented::set_init_pos_std(double std)
{
    params.init_params.pos_std = std;
}

void PositionUnscented::set_init_vel_std(double std)
{
    params.init_params.vel_std = std;
}

void PositionUnscented::set_init_accel_std(double std)
{
    params.init_params.accel_std = std;
}
