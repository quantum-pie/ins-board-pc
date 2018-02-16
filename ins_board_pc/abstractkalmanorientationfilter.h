#ifndef ABSTRACTKALMANPOSITIONFILTER_H
#define ABSTRACTKALMANPOSITIONFILTER_H

#include "abstractorientationfilter.h"

class AbstractKalmanOrientationFilter : public AbstractOrientationFilter
{
public:
    AbstractKalmanOrientationFilter(int accum_capacity) : AbstractOrientationFilter(accum_capacity) {}
    ~AbstractKalmanOrientationFilter() override {}

    virtual void set_proc_gyro_std(double std) = 0;
    virtual void set_proc_gyro_bias_std(double std) = 0;

    virtual void set_meas_accel_std(double std) = 0;
    virtual void set_meas_magn_std(double std) = 0;

    virtual void set_init_qs_std(double std) = 0;
    virtual void set_init_qx_std(double std) = 0;
    virtual void set_init_qy_std(double std) = 0;
    virtual void set_init_qz_std(double std) = 0;
    virtual void set_init_bias_std(double std) = 0;
};

#endif // ABSTRACTKALMANPOSITIONFILTER_H
