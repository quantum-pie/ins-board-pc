#include "poskalman.h"
#include "wmmwrapper.h"
#include "physconst.h"

#include <Eigen/Dense>

#include <QtMath>

const int PositionKalman::state_size = 9;
const int PositionKalman::measurement_size = 4;

PositionKalman::PositionKalman(const FilterParams & par)
    : AbstractKalmanPositionFilter(),
      params(par)
{
    x = NumVector(state_size);
    P = NumMatrix(state_size, state_size);
}

PositionKalman::~PositionKalman()
{

}

void PositionKalman::initialize(const FilterInput & z)
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

void PositionKalman::accumulate(const FilterInput &)
{

}

void PositionKalman::step(const FilterInput & z)
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

void PositionKalman::update(const FilterInput & z)
{
    NumMatrix F = create_transition_mtx(z);
    NumMatrix Q = create_proc_noise_cov_mtx(z.dt);

    x = F * x;
    P = F * P * F.transpose() + Q;

    NumVector z_pr(measurement_size);

    double lat, lon, alt;
    NumVector predicted_pos = get_position();
    calculate_geodetic(predicted_pos, lat, lon, alt);

    z_pr(0) = predicted_pos(0);
    z_pr(1) = predicted_pos(1);
    z_pr(2) = predicted_pos(2);

    NumVector predicted_v = get_velocity();
    calculate_velocity(predicted_v, z_pr(3));

    NumVector z_meas(measurement_size);
    z_meas << z.pos, z.v.norm();

    NumVector y = z_meas - z_pr;

    NumMatrix R = create_meas_noise_cov_mtx(lat, lon);
    NumMatrix H = create_meas_proj_mtx(predicted_v);

    NumMatrix S = H * P * H.transpose() + R;
    NumMatrix K = P * H.transpose() * S.inverse();

    x += K * y;

    NumMatrix tmp = NumMatrix::Identity(x.size(), x.size()) - K * H;
    P = tmp * P * tmp.transpose() + K * R * K.transpose();
}

NumMatrix PositionKalman::create_transition_mtx(const FilterInput & z) const
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

NumMatrix PositionKalman::create_proc_noise_cov_mtx(double dt) const
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

NumMatrix PositionKalman::create_meas_noise_cov_mtx(double lat, double lon) const
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

NumMatrix PositionKalman::create_meas_proj_mtx(const NumVector & v) const
{
    // 5
    NumMatrix Dpos_Dpos = NumMatrix::Identity(3, 3);

    // 6
    NumMatrix Dv_Dv(1, 3);
    double v_abs = v.norm();

    if(v_abs > 0)
    {
        Dv_Dv << v[0] / v_abs, v[1] / v_abs, v[2] / v_abs;
    }
    else
    {
        Dv_Dv << 0, 0, 0;
    }

    // Combine
    NumMatrix H(measurement_size, state_size);
    H <<   Dpos_Dpos, NumMatrix::Zero(3, 6),
            NumMatrix::Zero(1, 3), Dv_Dv, NumMatrix::Zero(1, 3);

    return H;
}

void PositionKalman::calculate_geodetic(const NumVector & position,
                                          double & lat, double & lon, double & alt) const
{
    WrapperWMM::instance().cartesian_to_geodetic(position, lat, lon, alt);
}

void PositionKalman::calculate_velocity(const NumVector & velocity, double & vel) const
{
    vel = velocity.norm();
}

NumVector PositionKalman::get_position() const
{
    return x.segment(0, 3);
}

NumVector PositionKalman::get_velocity() const
{
    return x.segment(3, 3);
}

NumVector PositionKalman::get_acceleration() const
{
    return x.segment(6, 3);
}

void PositionKalman::get_geodetic(double & lat, double & lon, double & alt) const
{
    calculate_geodetic(get_position(), lat, lon, alt);
}

void PositionKalman::set_proc_accel_std(double std)
{
    params.proc_params.accel_std = std;
}

void PositionKalman::set_meas_pos_std(double std)
{
    params.meas_params.gps_cep = std;
}

void PositionKalman::set_meas_vel_std(double std)
{
    params.meas_params.gps_vel_abs_std = std;
}

void PositionKalman::set_init_pos_std(double std)
{
    params.init_params.pos_std = std;
}

void PositionKalman::set_init_vel_std(double std)
{
    params.init_params.vel_std = std;
}

void PositionKalman::set_init_accel_std(double std)
{
    params.init_params.accel_std = std;
}
