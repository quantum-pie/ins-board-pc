#include "positionlkf.h"
#include "physconst.h"

#include <Eigen/Dense>

#include <QtMath>
#include <QDebug>

const int PositionLKF::state_size = 9;
const int PositionLKF::measurement_size = 6;

PositionLKF::PositionLKF(const FilterParams & par)
    : KalmanPositionFilter(par.track_history),
      params(par)
{
    x = NumVector(state_size);
    P = NumMatrix(state_size, state_size);
}

PositionLKF::~PositionLKF()
{

}

void PositionLKF::initialize_init_covar()
{
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

void PositionLKF::initialize(const FilterInput & z)
{
    KalmanPositionFilter::initialize(z);

    x[0] = z.pos[0];
    x[1] = z.pos[1];
    x[2] = z.pos[2];

    x[3] = z.v[0];
    x[4] = z.v[1];
    x[5] = z.v[2];

    x[6] = 0;
    x[7] = 0;
    x[8] = 0;

    initialize_init_covar();
}

void PositionLKF::update(const FilterInput & z)
{
    NumMatrix F = create_transition_mtx(z);
    NumMatrix Q = create_proc_noise_cov_mtx(z.dt);

    x = F * x;
    P = F * P * F.transpose() + Q;

    if(z.gps_fresh)
    {
        NumVector predicted_pos = get_position();
        NumVector geo = calculate_geodetic(predicted_pos);

        NumVector z_pr(measurement_size);
        z_pr << predicted_pos, get_velocity();

        NumVector z_meas(measurement_size);
        z_meas << z.pos, z.v;

        NumVector y = z_meas - z_pr;

        NumMatrix R = create_meas_noise_cov_mtx(geo);
        NumMatrix H = create_meas_proj_mtx();

        NumMatrix S = H * P * H.transpose() + R;
        NumMatrix K = P * H.transpose() * S.inverse();

        x += K * y;

        NumMatrix tmp = NumMatrix::Identity(x.size(), x.size()) - K * H;
        P = tmp * P * tmp.transpose() + K * R * K.transpose();

        PositionFilter::update(z);
    }
}

void PositionLKF::step(const FilterInput & z)
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

void PositionLKF::sim_step(const FilterInput & z)
{
    if(is_initialized())
    {
        sim_update(z);
    }
    else
    {
        sim_initialize(z);
    }
}

void PositionLKF::sim_initialize(const FilterInput & z)
{
    KalmanPositionFilter::initialize(z);

    double slat = std::sin(start_geo[0]);
    double clat = std::cos(start_geo[0]);
    double slon = std::sin(start_geo[1]);
    double clon = std::cos(start_geo[1]);

    double er = WrapperWMM::instance().earth_rad();
    x[0] = er * clat * clon;
    x[1] = er * clat * slon;
    x[2] = er * slat;

    double track_rad = -45.0 / 180 * M_PI;

    double vn = std::cos(track_rad);
    double ve = std::sin(track_rad);
    double vu = 0;

    NumVector enu(3);
    enu << ve, vn, vu;

    NumMatrix dcm_tr = WrapperWMM::instance().geodetic_to_dcm(start_geo).transpose();
    NumVector vc = dcm_tr * enu;

    x[3] = vc[0];
    x[4] = vc[1];
    x[5] = vc[2];

    x[6] = 0;
    x[7] = 0;
    x[8] = 0;

    initialize_init_covar();
}

void PositionLKF::sim_update(const FilterInput & z)
{
    double earth_rad = WrapperWMM::instance().earth_rad();

    NumVector geo(2);
    geo(0) = std::asin(x[2] / earth_rad);
    geo(1) = std::atan2(x[1], x[0]);

    double delta = 28 * z.dt / earth_rad;

    NumVector local_vel = WrapperWMM::instance().geodetic_to_dcm(geo) * get_velocity();
    double bearing = std::atan2(local_vel[0], local_vel[1]);

    double slat = std::sin(geo[0]);
    double clat = std::cos(geo[0]);
    double slon = std::sin(geo[1]);
    double clon = std::cos(geo[1]);
    double sdel = std::sin(delta);
    double cdel = std::cos(delta);
    double sbea = std::sin(bearing);
    double cbea = std::cos(bearing);

    geo[0] = std::asin(slat * cdel + clat * sdel * cbea);
    geo[1] += std::atan2(sbea * sdel * clat, cdel - slat * std::sin(geo[0]));

    if(geo[0] >= M_PI / 2 || geo[1] <= -M_PI)
    {
        geo = start_geo;
        x.segment(0, 3) = start_ecef;
    }

    slat = std::sin(geo[0]);
    clat = std::cos(geo[0]);
    slon = std::sin(geo[1]);
    clon = std::cos(geo[1]);

    double x_new = earth_rad * clat * clon;
    double y_new = earth_rad * clat * slon;
    double z_new = earth_rad * slat;

    double vx_new = (x_new - x[0]) / z.dt;
    double vy_new = (y_new - x[1]) / z.dt;
    double vz_new = (z_new - x[2]) / z.dt;

    double ax_new = (vx_new - x[3]) / z.dt;
    double ay_new = (vy_new - x[4]) / z.dt;
    double az_new = (vz_new - x[5]) / z.dt;

    x[0] = x_new;
    x[1] = y_new;
    x[2] = z_new;

    x[3] = vx_new;
    x[4] = vy_new;
    x[5] = vz_new;

    x[6] = ax_new;
    x[7] = ay_new;
    x[8] = az_new;

    PositionFilter::update(z);
}

void PositionLKF::bypass_step(const FilterInput & z)
{
    if(is_initialized())
    {
        x[0] = z.pos[0];
        x[1] = z.pos[1];
        x[2] = z.pos[2];

        x[3] = z.v[0];
        x[4] = z.v[1];
        x[5] = z.v[2];

        x[6] = 0;
        x[7] = 0;
        x[8] = 0;
    }
    else
    {
        KalmanPositionFilter::initialize(z);
    }
}

NumMatrix PositionLKF::create_transition_mtx(const FilterInput & z) const
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

NumMatrix PositionLKF::create_proc_noise_cov_mtx(double dt) const
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

NumMatrix PositionLKF::create_meas_noise_cov_mtx(const NumVector & geo) const
{
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

    R << Rp, NumMatrix::Zero(3, 3),
            NumMatrix::Zero(3, 3), NumMatrix::Identity(3, 3) * vel_variance;

    return R;
}

NumMatrix PositionLKF::create_meas_proj_mtx() const
{
    // 5
    NumMatrix Dpos_Dpos = NumMatrix::Identity(3, 3);

    // 6
    NumMatrix Dv_Dv = NumMatrix::Identity(3, 3);

    // Combine
    NumMatrix H(measurement_size, state_size);
    H <<   Dpos_Dpos, NumMatrix::Zero(3, 6),
            NumMatrix::Zero(3, 3), Dv_Dv, NumMatrix::Zero(3, 3);

    return H;
}

NumVector PositionLKF::get_position() const
{
    return x.segment(0, 3);
}

NumVector PositionLKF::get_velocity() const
{
    return x.segment(3, 3);
}

NumVector PositionLKF::get_acceleration() const
{
    return x.segment(6, 3);
}

void PositionLKF::set_proc_accel_std(double std)
{
    params.proc_params.accel_std = std;
}

void PositionLKF::set_meas_pos_std(double std)
{
    params.meas_params.gps_cep = std;
}

void PositionLKF::set_meas_vel_std(double std)
{
    params.meas_params.gps_vel_std = std;
}

void PositionLKF::set_init_pos_std(double std)
{
    params.init_params.pos_std = std;
}

void PositionLKF::set_init_vel_std(double std)
{
    params.init_params.vel_std = std;
}

void PositionLKF::set_init_accel_std(double std)
{
    params.init_params.accel_std = std;
}
