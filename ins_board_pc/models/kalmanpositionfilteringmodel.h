#ifndef KALMANPOSITIONFILTERINGMODEL_H
#define KALMANPOSITIONFILTERINGMODEL_H

#include "positionfilteringmodel.h"
#include "filtering/public_interfaces/IKalmanPositionFilter.h"

struct KalmanPositionFilteringModel : PositionFilteringModel<IKalmanPositionFilter>
{
    KalmanPositionFilteringModel()
        : PositionFilteringModel<IKalmanPositionFilter>() {}

    void set_proc_accel_std(double std)
    {
        filtering_strategy->set_proc_accel_std(std);
    }

    void set_meas_pos_std(double std)
    {
        filtering_strategy->set_meas_pos_std(std);
    }

    void set_meas_vel_std(double std)
    {
        filtering_strategy->set_meas_vel_std(std);
    }

    void set_init_pos_std(double std)
    {
        filtering_strategy->set_init_pos_std(std);
    }

    void set_init_vel_std(double std)
    {
        filtering_strategy->set_init_vel_std(std);
    }

    void set_init_accel_std(double std)
    {
        filtering_strategy->set_init_accel_std(std);
    }

    double get_proc_accel_std() const
    {
        return filtering_strategy->get_proc_accel_std();
    }

    double get_meas_pos_std() const
    {
        return filtering_strategy->get_meas_pos_std();
    }

    double get_meas_vel_std() const
    {
        return filtering_strategy->get_meas_vel_std();
    }

    double get_init_pos_std() const
    {
        return filtering_strategy->get_init_pos_std();
    }

    double get_init_vel_std() const
    {
        return filtering_strategy->get_init_vel_std();
    }

    double get_init_accel_std() const
    {
        return filtering_strategy->get_init_accel_std();
    }
};

#endif // KALMANPOSITIONFILTERINGMODEL_H
