#include "kalmanorientationfilteringmodel.h"

void KalmanOrientationFilteringModel::set_proc_gyro_std(double std)
{
    filtering_strategy->set_proc_gyro_std(std);
}

void KalmanOrientationFilteringModel::set_proc_gyro_bias_std(double std)
{
    filtering_strategy->set_proc_gyro_bias_std(std);
}

void KalmanOrientationFilteringModel::set_meas_accel_std(double std)
{
    filtering_strategy->set_meas_accel_std(std);
}

void KalmanOrientationFilteringModel::set_meas_magn_std(double std)
{
    filtering_strategy->set_meas_magn_std(std);
}

void KalmanOrientationFilteringModel::set_init_qs_std(double std)
{
    filtering_strategy->set_init_qs_std(std);
}

void KalmanOrientationFilteringModel::set_init_qx_std(double std)
{
    filtering_strategy->set_init_qx_std(std);
}

void KalmanOrientationFilteringModel::set_init_qy_std(double std)
{
    filtering_strategy->set_init_qy_std(std);
}

void KalmanOrientationFilteringModel::set_init_qz_std(double std)
{
    filtering_strategy->set_init_qz_std(std);
}

void KalmanOrientationFilteringModel::set_init_bias_std(double std)
{
    filtering_strategy->set_init_bias_std(std);
}

double KalmanOrientationFilteringModel::get_proc_gyro_std() const
{
    return filtering_strategy->get_proc_gyro_std();
}

double KalmanOrientationFilteringModel::get_proc_gyro_bias_std() const
{
    return filtering_strategy->get_proc_gyro_bias_std();
}

double KalmanOrientationFilteringModel::get_meas_accel_std() const
{
    return filtering_strategy->get_meas_accel_std();
}

double KalmanOrientationFilteringModel::get_meas_magn_std() const
{
    return filtering_strategy->get_meas_magn_std();
}

double KalmanOrientationFilteringModel::get_init_qs_std() const
{
    return filtering_strategy->get_init_qs_std();
}

double KalmanOrientationFilteringModel::get_init_qx_std() const
{
    return filtering_strategy->get_init_qx_std();
}

double KalmanOrientationFilteringModel::get_init_qy_std() const
{
    return filtering_strategy->get_init_qy_std();
}

double KalmanOrientationFilteringModel::get_init_qz_std() const
{
    return filtering_strategy->get_init_qz_std();
}

double KalmanOrientationFilteringModel::get_init_bias_std() const
{
    return filtering_strategy->get_init_bias_std();
}
