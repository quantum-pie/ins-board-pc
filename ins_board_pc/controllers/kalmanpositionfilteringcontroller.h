#ifndef KALMANPOSITIONFILTERINGCONTROLLER_H
#define KALMANPOSITIONFILTERINGCONTROLLER_H

#include "filtering/filters/generickalman.h"
#include "controllers/filteringcontroller.h"
#include "models/kalmanpositionfilteringmodel.h"

class KalmanPositionFilteringController : public BaseFilteringController
{
public:
    explicit KalmanPositionFilteringController(KalmanPositionFilteringModel & model);
    void handle_kp_strategy(IKalmanPositionFilter * other_filter);

    using BaseFilteringController::handle_input;
    using BaseFilteringController::handle_start;

public slots:
    void handle_kp_strategy(int combobox_idx);
    void on_proc_accel_std(double std);
    void on__meas_pos_std(double std);
    void on_meas_vel_std(double std);
    void on_init_pos_std(double std);
    void on_init_vel_std(double std);
    void on_init_accel_std(double std);

private:
    KalmanPositionFilteringModel & model;

    PositionEKF ekf_filter;
    PositionUKF ukf_filter;
};

#endif // KALMANPOSITIONFILTERINGCONTROLLER_H
