#ifndef KALMANORIENTATIONFILTERINGCONTROLLER_H
#define KALMANORIENTATIONFILTERINGCONTROLLER_H

#include "filtering/filters/generickalman.h"
#include "controllers/basefilteringcontroller.h"
#include "models/kalmanorientationfilteringmodel.h"

class KalmanOrientationFilteringController :
        public BaseFilteringController<KalmanOrientationFilteringModel>
{
public:
    using model_type = KalmanOrientationFilteringModel;
    using base_type = BaseFilteringController<model_type>;
    using base_type::handle_strategy;

    KalmanOrientationFilteringController(model_type & model)
        : base_type{ model } {}

public slots:
    void handle_strategy(int combobox_idx)
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

    void on_proc_gyro_std(double std)
    {
        model.set_proc_gyro_std(std);
    }

    void on_proc_gyro_bias_std(double std)
    {
        model.set_proc_gyro_bias_std(std);
    }

    void on_meas_accel_std(double std)
    {
        model.set_meas_accel_std(std);
    }

    void on_meas_magn_std(double std)
    {
        model.set_meas_magn_std(std);
    }

    void on_init_qs_std(double std)
    {
        model.set_init_qs_std(std);
    }

    void on_init_qx_std(double std)
    {
        model.set_init_qx_std(std);
    }

    void on_init_qy_std(double std)
    {
        model.set_init_qy_std(std);
    }

    void on_init_qz_std(double std)
    {
        model.set_init_qz_std(std);
    }

    void on_init_bias_std(double std)
    {
        model.set_init_bias_std(std);
    }
private:
    OrientationEKF ekf_filter;
    OrientationUKF ukf_filter;
};

#endif // KALMANORIENTATIONFILTERINGCONTROLLER_H
