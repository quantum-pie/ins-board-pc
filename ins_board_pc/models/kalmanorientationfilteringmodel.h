#ifndef KALMANORIENTATIONFILTERINGMODEL_H
#define KALMANORIENTATIONFILTERINGMODEL_H

#include "models/orientationfilteringmodel.h"
#include "filtering/public_interfaces/IKalmanOrientationFilter.h"

struct KalmanOrientationFilteringModel : OrientationFilteringModel<IKalmanOrientationFilter>
{
    void set_proc_gyro_std(double std);
    void set_proc_gyro_bias_std(double std);
    void set_meas_accel_std(double std);
    void set_meas_magn_std(double std);
    void set_init_qs_std(double std);
    void set_init_qx_std(double std);
    void set_init_qy_std(double std);
    void set_init_qz_std(double std);
    void set_init_bias_std(double std);

    double get_proc_gyro_std() const;
    double get_proc_gyro_bias_std() const;
    double get_meas_accel_std() const;
    double get_meas_magn_std() const;
    double get_init_qs_std() const;
    double get_init_qx_std() const;
    double get_init_qy_std() const;
    double get_init_qz_std() const;
    double get_init_bias_std() const;

    template<typename OtherKalmanOrientationFilter>
    operator KalmanOrientationFilteringModel<OtherKalmanOrientationFilter>() const
    {
        KalmanOrientationFilteringModel<OtherKalmanOrientationFilter> res;
        res.set_strategy(filtering_strategy);
        return res;
    }
};

#endif // KALMANORIENTATIONFILTERINGMODEL_H
