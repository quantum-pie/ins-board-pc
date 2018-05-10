#include "orientationekf.h"
#include "geometry.h"
#include "quaternion.h"
#include "quatutils.h"
#include "packets.h"

#include <Eigen/Dense>

using namespace quat;
using namespace geom;

OrientationEKF::OrientationEKF(const FilterParams & par)
    : KalmanOrientationFilterBase{ par },
      is_initialized{ false },
      bias_ctrl{ buffer_size },
      x{ state_type::Zero() },
      P{ P_type::Identity() }
{}

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
    bias_ctrl.set_sampling(buffer_size);
}

Quaternion OrientationEKF::get_orientation_quaternion() const
{
    return static_cast<vector_form>(x.segment<4>(0));
}

Vector3D OrientationEKF::get_gyro_bias() const
{
    return x.segment<3>(4);
}

void OrientationEKF::initialize(const FilterInput & z)
{
    // TODO check and correct if wrong
    x.segment<4>(0) = static_cast<vector_form>( accel_magn_quat(z.a, z.m, earth_model.magnetic_declination(z.geo, z.day)).conjugate() );
    x.segment<3>(4) = bias_ctrl.get_mean();

    P = create_init_cov_mtx();

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
        Vector3D predicted_acc = predict_accelerometer(get_orientation_quaternion(), earth_model.gravity(z.geo));
        Vector3D predicted_magn = predict_magnetometer(get_orientation_quaternion(), earth_model.magnetic_vector(z.geo, z.day));

        meas_type z_pr;
        z_pr << predicted_acc, predicted_magn;

        meas_type z_meas;
        z_meas << z.a, z.m;

        auto y = z_meas - z_pr;

        R_type R = create_meas_noise_cov_mtx(earth_model.magnetic_magnitude(z.geo, z.day));
        H_type H = create_meas_proj_mtx(earth_model.magnetic_vector(z.geo, z.day), earth_model.gravity(z.geo));

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
    x.segment<4>(0) = static_cast<vector_form>(get_orientation_quaternion().normalize());
}
