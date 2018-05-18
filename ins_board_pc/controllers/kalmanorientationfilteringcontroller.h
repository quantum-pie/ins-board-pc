#ifndef KALMANORIENTATIONFILTERINGCONTROLLER_H
#define KALMANORIENTATIONFILTERINGCONTROLLER_H

#include "filtering/filters/generickalman.h"
#include "controllers/basefilteringcontroller.h"
#include "models/kalmanorientationfilteringmodel.h"

class KalmanOrientationFilteringController : private BaseFilteringController
{
public:
    explicit KalmanOrientationFilteringController(KalmanOrientationFilteringModel & model);
    void handle_ko_strategy(IKalmanOrientationFilter * filter);

    using BaseFilteringController::handle_input;
    using BaseFilteringController::handle_start;

public slots:
    void handle_ko_strategy(int combobox_idx);
    void on_proc_gyro_std(double std);
    void on_proc_gyro_bias_std(double std);
    void on_meas_accel_std(double std);
    void on_meas_magn_std(double std);
    void on_init_qs_std(double std);
    void on_init_qx_std(double std);
    void on_init_qy_std(double std);
    void on_init_qz_std(double std);
    void on_init_bias_std(double std);

private:
    KalmanOrientationFilteringModel & model;

    OrientationEKF ekf_filter;
    OrientationUKF ukf_filter;
};

#endif // KALMANORIENTATIONFILTERINGCONTROLLER_H
