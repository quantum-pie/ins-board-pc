#include "models/kalmanpositionfilteringmodel.h"

void KalmanPositionFilteringModel::set_strategy(IKalmanPositionFilter * other_filter)
{
    filtering_strategy = other_filter;
    emit strategy_changed();
}

void KalmanPositionFilteringModel::do_step(const FilterInput & z)
{
    filtering_strategy->step(z);
}

void KalmanPositionFilteringModel::do_reset()
{
    filtering_strategy->reset();
}

Vector3D KalmanPositionFilteringModel::do_get_cartesian() const
{
    return filtering_strategy->get_cartesian();
}

Ellipsoid KalmanPositionFilteringModel::do_get_ellipsoid() const
{
    return filtering_strategy->get_ellipsoid();
}

Vector3D KalmanPositionFilteringModel::do_get_velocity() const
{
    return filtering_strategy->get_velocity();
}

Vector3D KalmanPositionFilteringModel::do_get_acceleration() const
{
    return filtering_strategy->get_acceleration();
}

void KalmanPositionFilteringModel::do_set_proc_accel_std(double std)
{
    filtering_strategy->set_proc_accel_std(std);
}

void KalmanPositionFilteringModel::do_set_meas_pos_std(double std)
{
    filtering_strategy->set_meas_pos_std(std);
}

void KalmanPositionFilteringModel::do_set_meas_vel_std(double std)
{
    filtering_strategy->set_meas_vel_std(std);
}

void KalmanPositionFilteringModel::do_set_init_pos_std(double std)
{
    filtering_strategy->set_init_pos_std(std);
}

void KalmanPositionFilteringModel::do_set_init_vel_std(double std)
{
    filtering_strategy->set_init_vel_std(std);
}

void KalmanPositionFilteringModel::do_set_init_accel_std(double std)
{
    filtering_strategy->set_init_accel_std(std);
}

double KalmanPositionFilteringModel::do_get_proc_accel_std() const
{
    return filtering_strategy->get_proc_accel_std();
}

double KalmanPositionFilteringModel::do_get_meas_pos_std() const
{
    return filtering_strategy->get_meas_pos_std();
}

double KalmanPositionFilteringModel::do_get_meas_vel_std() const
{
    return filtering_strategy->get_meas_vel_std();
}

double KalmanPositionFilteringModel::do_get_init_pos_std() const
{
    return filtering_strategy->get_init_pos_std();
}

double KalmanPositionFilteringModel::do_get_init_vel_std() const
{
    return filtering_strategy->get_init_vel_std();
}

double KalmanPositionFilteringModel::do_get_init_accel_std() const
{
    return filtering_strategy->get_init_accel_std();
}
