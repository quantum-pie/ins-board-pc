#ifndef ABSTRACTPOSITIONKALMANFILTER_H
#define ABSTRACTPOSITIONKALMANFILTER_H

#include "abstractpositionfilter.h"

class AbstractKalmanPositionFilter : public AbstractPositionFilter
{
public:
    AbstractKalmanPositionFilter() : AbstractPositionFilter() {}
    ~AbstractKalmanPositionFilter() override {}

    virtual void set_proc_accel_std(double std) = 0;

    virtual void set_meas_pos_std(double std) = 0;
    virtual void set_meas_vel_std(double std) = 0;

    virtual void set_init_pos_std(double std) = 0;
    virtual void set_init_vel_std(double std) = 0;
    virtual void set_init_accel_std(double std) = 0;
};

#endif // ABSTRACTPOSITIONKALMANFILTER_H
