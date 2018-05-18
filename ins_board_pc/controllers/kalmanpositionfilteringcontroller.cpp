#include "controllers/kalmanpositionfilteringcontroller.h"

KalmanPositionFilteringController::KalmanPositionFilteringController(KalmanPositionFilteringModel & model)
    : BaseFilteringController{ model }, model{ model } {}

void KalmanPositionFilteringController::handle_kp_strategy(IKalmanPositionFilter * other_filter)
{
    handle_strategy(other_filter);
}

void KalmanPositionFilteringController::handle_kp_strategy(int combobox_idx)
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

void KalmanPositionFilteringController::on_proc_accel_std(double std)
{
    model.set_proc_accel_std(std);
}

void KalmanPositionFilteringController::on__meas_pos_std(double std)
{
    model.set_meas_pos_std(std);
}

void KalmanPositionFilteringController::on_meas_vel_std(double std)
{
    model.set_meas_vel_std(std);
}

void KalmanPositionFilteringController::on_init_pos_std(double std)
{
    model.set_init_pos_std(std);
}

void KalmanPositionFilteringController::on_init_vel_std(double std)
{
    model.set_init_vel_std(std);
}

void KalmanPositionFilteringController::on_init_accel_std(double std)
{
    model.set_init_accel_std(std);
}
