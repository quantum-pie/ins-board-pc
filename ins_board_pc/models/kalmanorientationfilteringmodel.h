#ifndef KALMANORIENTATIONFILTERINGMODEL_H
#define KALMANORIENTATIONFILTERINGMODEL_H

#include "models/orientationfilteringmodel.h"
#include "filtering/public_interfaces/IKalmanOrientationFilter.h"

struct KalmanOrientationFilteringModel : OrientationFilteringModel<IKalmanOrientationFilter>
{
    KalmanOrientationFilteringModel()
        : OrientationFilteringModel<IKalmanOrientationFilter>() {}

    void set_proc_gyro_std(double std)
    {
        filtering_strategy->set_proc_gyro_std(std);
    }

    void set_proc_gyro_bias_std(double std)
    {
        filtering_strategy->set_proc_gyro_bias_std(std);
    }

    void set_meas_accel_std(double std)
    {
        filtering_strategy->set_meas_accel_std(std);
    }

    void set_meas_magn_std(double std)
    {
        filtering_strategy->set_meas_magn_std(std);
    }

    void set_init_qs_std(double std)
    {
        filtering_strategy->set_init_qs_std(std);
    }

    void set_init_qx_std(double std)
    {
        filtering_strategy->set_init_qx_std(std);
    }

    void set_init_qy_std(double std)
    {
        filtering_strategy->set_init_qy_std(std);
    }

    void set_init_qz_std(double std)
    {
        filtering_strategy->set_init_qz_std(std);
    }

    void set_init_bias_std(double std)
    {
        filtering_strategy->set_init_bias_std(std);
    }

    double get_proc_gyro_std() const
    {
        return filtering_strategy->get_proc_gyro_std();
    }

    double get_proc_gyro_bias_std() const
    {
        return filtering_strategy->get_proc_gyro_bias_std();
    }

    double get_meas_accel_std() const
    {
        return filtering_strategy->get_meas_accel_std();
    }

    double get_meas_magn_std() const
    {
        return filtering_strategy->get_meas_magn_std();
    }

    double get_init_qs_std() const
    {
        return filtering_strategy->get_init_qs_std();
    }

    double get_init_qx_std() const
    {
        return filtering_strategy->get_init_qx_std();
    }

    double get_init_qy_std() const
    {
        return filtering_strategy->get_init_qy_std();
    }

    double get_init_qz_std() const
    {
        return filtering_strategy->get_init_qz_std();
    }

    double get_init_bias_std() const
    {
        return filtering_strategy->get_init_bias_std();
    }
};

#endif // KALMANORIENTATIONFILTERINGMODEL_H
