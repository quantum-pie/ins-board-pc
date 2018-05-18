#ifndef KALMANPOSITIONFILTERINGMODEL_H
#define KALMANPOSITIONFILTERINGMODEL_H

#include "models/IFilteringModel.h"
#include "filtering/public_interfaces/IKalmanPositionFilter.h"

struct KalmanPositionFilteringModel : IFilteringModel<IKalmanPositionFilter>
{
    KalmanPositionFilteringModel(IKalmanPositionFilter * filter)
        : filtering_strategy{ filter } {}

    void set_strategy(IKalmanPositionFilter * other_filter);

signals:
    void strategy_changed() const;

private:
    void do_step(const FilterInput & z) override;
    void do_reset() override;

    Vector3D do_get_cartesian() const override;
    Ellipsoid do_get_ellipsoid() const override;
    Vector3D do_get_velocity() const override;
    Vector3D do_get_acceleration() const override;

    void do_set_proc_accel_std(double std) override;
    void do_set_meas_pos_std(double std) override;
    void do_set_meas_vel_std(double std) override;
    void do_set_init_pos_std(double std) override;
    void do_set_init_vel_std(double std) override;
    void do_set_init_accel_std(double std) override;

    double do_get_proc_accel_std() const override;
    double do_get_meas_pos_std() const override;
    double do_get_meas_vel_std() const override;
    double do_get_init_pos_std() const override;
    double do_get_init_vel_std() const override;
    double do_get_init_accel_std() const override;

    IKalmanPositionFilter * filtering_strategy;
};

#endif // KALMANPOSITIONFILTERINGMODEL_H
