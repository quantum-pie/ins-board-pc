#ifndef KALMANPOSITIONFILTERINGCONTROLLER_H
#define KALMANPOSITIONFILTERINGCONTROLLER_H

#include "filtering/filters/generickalman.h"
#include "controllers/basefilteringcontroller.h"
#include "models/kalmanpositionfilteringmodel.h"

class KalmanPositionFilteringController :
        public BaseFilteringController<KalmanPositionFilteringModel>
{
public:
    using model_type = KalmanPositionFilteringModel;
    using base_type = BaseFilteringController<model_type>;
    using base_type::handle_strategy;

    KalmanPositionFilteringController(model_type & model)
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

    void on_proc_accel_std(double std)
    {
        model.set_proc_accel_std(std);
    }

    void on__meas_pos_std(double std)
    {
        model.set_meas_pos_std(std);
    }

    void on_meas_vel_std(double std)
    {
        model.set_meas_vel_std(std);
    }

    void on_init_pos_std(double std)
    {
        model.set_init_pos_std(std);
    }

    void on_init_vel_std(double std)
    {
        model.set_init_vel_std(std);
    }

    void on_init_accel_std(double std)
    {
        model.set_init_accel_std(std);
    }

private:
    PositionEKF ekf_filter;
    PositionUKF ukf_filter;
};

#endif // KALMANPOSITIONFILTERINGCONTROLLER_H
