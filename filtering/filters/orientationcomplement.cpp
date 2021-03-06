#include "orientationcomplement.h"
#include "quaternion.h"
#include "quatutils.h"
#include "qualitycontrol.h"
#include "earth.h"
#include "geometry.h"
#include "packets.h"

using namespace quat;
using namespace geom;

struct OrientationCF::Impl
{
    // Useful type aliases
    using V_type = quat::skew_type;
    using D_type = quat::delta_type;

    Impl()
        : is_initialized { false },
          bias_ctrl{ accum_size },
          accel_ctrl{ accum_size },
          magn_ctrl{ accum_size },
          state_bias { Vector3D::Zero() },
          params{ default_params }
    {}


    void step_uninitialized(const FilterInput & z)
    {
        bias_ctrl.update(z.w);
        accel_ctrl.update(z.a);
        magn_ctrl.update(z.m);
    }

    void initialize()
    {
        state_quat = accel_magn_quat(accel_ctrl.get_mean(), magn_ctrl.get_mean()).conjugate();
        state_bias = bias_ctrl.get_mean();

        is_initialized = true;
        bias_ctrl.set_sampling(0); // free memory
        accel_ctrl.set_sampling(0);
        magn_ctrl.set_sampling(0);
    }

    void step_initialized(const FilterInput & z)
    {
        const double dt_2 = z.dt / 2;

        auto V = skew_symmetric(z.w);

        V *= dt_2;
        V += V_type::Identity();

        auto K = state_quat.delta_mtx(dt_2);
		
		// propagate quaternion
        state_quat = (V * static_cast<vector_form>(state_quat) + K * state_bias).eval();
        state_quat.normalize();

        // residual quaternion
        Quaternion qerr = accel_magn_quat(z.a, z.m) * state_quat;
        auto bias_corr = qerr.vector_part();
        bias_corr[2] /= 100;

        // update bias
        state_bias += params.bias_gain * bias_corr;

        Vector3D a_norm = z.a / z.a.norm();

        // predict gravity
        Vector3D g_pred = state_quat.dcm() * a_norm;
        Quaternion qacc_delta = acceleration_quat(g_pred).conjugate();
        Quaternion qacc_corr = lerp(Quaternion::identity, qacc_delta, calculate_gain(z.a));
        state_quat = qacc_corr * state_quat;

        // predict magnetic vector
        Vector3D mag_pred = state_quat.dcm() * z.m;
        Quaternion qmag_delta = magnetometer_quat(mag_pred).conjugate();
        Quaternion qmag_corr = lerp(Quaternion::identity, qmag_delta, params.static_magn_gain);
        state_quat = qmag_corr * state_quat;

        de_declinator = z_rotator(earth_model.magnetic_declination(z.geo, z.day)).conjugate();
    }

    double calculate_gain(const Vector3D & accel) const
    {
        double error_metric = std::fabs(accel.norm() - 1);
        return params.static_accel_gain * gain_factor(error_metric);
    }

    double gain_factor(double error_metric) const
    {
        if(error_metric < 0.1)
        {
            return 1.0;
        }
        else if(error_metric < 0.2)
        {
            return 2.0 - 10.0 * error_metric;
        }
        else
        {
            return 0.0;
        }
    }

    bool is_initialized;
    QualityControl<Vector3D> bias_ctrl;
    QualityControl<Vector3D> accel_ctrl;
    QualityControl<Vector3D> magn_ctrl;
    const Earth earth_model;

    Quaternion state_quat;
    Quaternion de_declinator;
    Vector3D state_bias;

    struct FilterParams
    {
        double static_accel_gain;   //!< Static accelerometer measurements gain.
        double static_magn_gain;    //!< Static magnetometer measurements gain.
        double bias_gain;			//!< Gyroscope bias gain.
    } params;

    static const std::size_t accum_size; //!< Size of filter input accumulator.
    static const FilterParams default_params;
};

const std::size_t OrientationCF::Impl::accum_size { 500 };
const OrientationCF::Impl::FilterParams OrientationCF::Impl::default_params { 0.05, 0.001, 0.00001 };

OrientationCF::OrientationCF()
    : pimpl{ std::make_unique<Impl>() }
{}

OrientationCF::~OrientationCF() = default;

void OrientationCF::do_reset()
{
    pimpl->is_initialized = false;
    pimpl->bias_ctrl.set_sampling(Impl::accum_size);
    pimpl->accel_ctrl.set_sampling(Impl::accum_size);
    pimpl->magn_ctrl.set_sampling(Impl::accum_size);
}

void OrientationCF::do_step(const FilterInput & z)
{
    if(pimpl->is_initialized)
    {
        pimpl->step_initialized(z);
    }
    else if(pimpl->bias_ctrl.is_saturated())
    {
        pimpl->initialize();
    }
    else
    {
        pimpl->step_uninitialized(z);
    }
}

Quaternion OrientationCF::do_get_orientation_quaternion() const
{
    return pimpl->de_declinator * pimpl->state_quat;
}

Vector3D OrientationCF::do_get_gyro_bias() const
{
    return pimpl->state_bias;
}

void OrientationCF::do_set_static_accel_gain(double gain)
{
    pimpl->params.static_accel_gain = gain;
}

void OrientationCF::do_set_static_magn_gain(double gain)
{
    pimpl->params.static_magn_gain = gain;
}

void OrientationCF::do_set_bias_gain(double gain)
{
    pimpl->params.bias_gain = gain;
}

double OrientationCF::do_get_static_accel_gain() const
{
    return pimpl->params.static_accel_gain;
}

double OrientationCF::do_get_static_magn_gain() const
{
    return pimpl->params.static_magn_gain;
}

double OrientationCF::do_get_bias_gain() const
{
    return pimpl->params.bias_gain;
}
