#ifndef KALMANPOSITIONFILTERINGMODEL_H
#define KALMANPOSITIONFILTERINGMODEL_H

#include "models/positionfilteringmodel.h"
#include "filtering/public_interfaces/IKalmanPositionFilter.h"

class KalmanPositionFilteringModel : public PositionFilteringModel
{
public:
    explicit KalmanPositionFilteringModel(IKalmanPositionFilter * wrapee)
        : PositionFilteringModel{ wrapee }, wrapee{ wrapee } {}

    ~KalmanPositionFilteringModel() override = default;

    void set_proc_accel_std(double std)
    {
        wrapee->set_proc_accel_std(std);
    }

    void set_meas_pos_std(double std)
    {
        wrapee->set_meas_pos_std(std);
    }

    void set_meas_vel_std(double std)
    {
        wrapee->set_meas_vel_std(std);
    }

    void set_init_pos_std(double std)
    {
        wrapee->set_init_pos_std(std);
    }

    void set_init_vel_std(double std)
    {
        wrapee->set_init_vel_std(std);
    }

    void set_init_accel_std(double std)
    {
        wrapee->set_init_accel_std(std);
    }

    double get_proc_accel_std() const
    {
        return wrapee->get_proc_accel_std();
    }

    double get_meas_pos_std() const
    {
        return wrapee->get_meas_pos_std();
    }

    double get_meas_vel_std() const
    {
        return wrapee->get_meas_vel_std();
    }

    double get_init_pos_std() const
    {
        return wrapee->get_init_pos_std();
    }

    double get_init_vel_std() const
    {
        return wrapee->get_init_vel_std();
    }

    double get_init_accel_std() const
    {
        return wrapee->get_init_accel_std();
    }

private:
    IKalmanPositionFilter * wrapee;
};

#endif // KALMANPOSITIONFILTERINGMODEL_H
