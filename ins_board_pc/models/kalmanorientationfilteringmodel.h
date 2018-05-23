#ifndef KALMANORIENTATIONFILTERINGMODEL_H
#define KALMANORIENTATIONFILTERINGMODEL_H

#include "models/orientationfilteringmodel.h"
#include "filtering/public_interfaces/IKalmanOrientationFilter.h"

class KalmanOrientationFilteringModel : public OrientationFilteringModel
{
public:
    explicit KalmanOrientationFilteringModel(IKalmanOrientationFilter * wrapee)
        : OrientationFilteringModel{ wrapee }, wrapee{ wrapee } {}

    ~KalmanOrientationFilteringModel() override = default;

    void set_proc_gyro_std(double std)
    {
        wrapee->set_proc_gyro_std(std);
    }

    void set_proc_gyro_bias_std(double std)
    {
        wrapee->set_proc_gyro_bias_std(std);
    }

    void set_meas_accel_std(double std)
    {
        wrapee->set_meas_accel_std(std);
    }

    void set_meas_magn_std(double std)
    {
        wrapee->set_meas_magn_std(std);
    }

    void set_init_qs_std(double std)
    {
        wrapee->set_init_qs_std(std);
    }

    void set_init_qx_std(double std)
    {
        wrapee->set_init_qx_std(std);
    }

    void set_init_qy_std(double std)
    {
        wrapee->set_init_qy_std(std);
    }

    void set_init_qz_std(double std)
    {
        wrapee->set_init_qz_std(std);
    }

    void set_init_bias_std(double std)
    {
        wrapee->set_init_bias_std(std);
    }

    double get_proc_gyro_std() const
    {
        return wrapee->get_proc_gyro_std();
    }

    double get_proc_gyro_bias_std() const
    {
        return wrapee->get_proc_gyro_bias_std();
    }

    double get_meas_accel_std() const
    {
        return wrapee->get_meas_accel_std();
    }

    double get_meas_magn_std() const
    {
        return wrapee->get_meas_magn_std();
    }

    double get_init_qs_std() const
    {
        return wrapee->get_init_qs_std();
    }

    double get_init_qx_std() const
    {
        return wrapee->get_init_qx_std();
    }

    double get_init_qy_std() const
    {
        return wrapee->get_init_qy_std();
    }

    double get_init_qz_std() const
    {
        return wrapee->get_init_qz_std();
    }

    double get_init_bias_std() const
    {
        return wrapee->get_init_bias_std();
    }

private:
    IKalmanOrientationFilter * wrapee;
};

#endif // KALMANORIENTATIONFILTERINGMODEL_H
