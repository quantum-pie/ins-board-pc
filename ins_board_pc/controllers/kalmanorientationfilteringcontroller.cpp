#include "controllers/kalmanorientationfilteringcontroller.h"

KalmanOrientationFilteringController::KalmanOrientationFilteringController(KalmanOrientationFilteringModel & model)
    : BaseFilteringController{ model }, model{ model } {}

void KalmanOrientationFilteringController::handle_ko_strategy(IKalmanOrientationFilter * filter)
{
    handle_strategy(filter);
}

void KalmanOrientationFilteringController::handle_ko_strategy(int combobox_idx)
{
    if(combobox_idx == 0)
    {
        model.set_strategy(&ekf_filter);
    }
    else if(combobox_idx == 1)
    {
        model.set_strategy(&ukf_filter);
    }
}

void KalmanOrientationFilteringController::on_proc_gyro_std(double std)
{
    model.set_proc_gyro_std(std);
}

void KalmanOrientationFilteringController::on_proc_gyro_bias_std(double std)
{
    model.set_proc_gyro_bias_std(std);
}

void KalmanOrientationFilteringController::on_meas_accel_std(double std)
{
    model.set_meas_accel_std(std);
}

void KalmanOrientationFilteringController::on_meas_magn_std(double std)
{
    model.set_meas_magn_std(std);
}

void KalmanOrientationFilteringController::on_init_qs_std(double std)
{
    model.set_init_qs_std(std);
}

void KalmanOrientationFilteringController::on_init_qx_std(double std)
{
    model.set_init_qx_std(std);
}

void KalmanOrientationFilteringController::on_init_qy_std(double std)
{
    model.set_init_qy_std(std);
}

void KalmanOrientationFilteringController::on_init_qz_std(double std)
{
    model.set_init_qz_std(std);
}

void KalmanOrientationFilteringController::on_init_bias_std(double std)
{
    model.set_init_bias_std(std);
}
