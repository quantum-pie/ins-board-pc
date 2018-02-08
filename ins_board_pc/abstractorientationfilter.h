#ifndef ABSTRACTORIENTATIONFILTER_H
#define ABSTRACTORIENTATIONFILTER_H

#include "abstractfilter.h"
#include "qualitycontrol.h"

class AbstractOrientationFilter : public virtual AbstractFilter
{
public:
    AbstractOrientationFilter(int accum_capacity)
        : AbstractFilter(),
          bias_x_ctrl(accum_capacity), bias_y_ctrl(accum_capacity), bias_z_ctrl(accum_capacity)
    {
        reset_this();
    }

    ~AbstractOrientationFilter() override {}

    void reset() override
    {
        AbstractFilter::reset();
        reset_this();
    }

    virtual NumVector get_orientation_quaternion() = 0;
    virtual NumVector get_gyro_bias() = 0;
    virtual void get_rpy(double & roll, double & pitch, double & yaw) = 0;

protected:
    void accumulate(const FilterInput & z) override
    {
        bias_x_ctrl.update(z.w[0]);
        bias_y_ctrl.update(z.w[1]);
        bias_z_ctrl.update(z.w[2]);
    }

    bool bias_estimated()
    {
        return bias_x_ctrl.is_saturated() &&
                bias_y_ctrl.is_saturated() &&
                bias_z_ctrl.is_saturated();
    }

    virtual void normalize_state() = 0;

    QualityControl bias_x_ctrl;
    QualityControl bias_y_ctrl;
    QualityControl bias_z_ctrl;

private:
    void reset_this()
    {
        bias_x_ctrl.reset();
        bias_y_ctrl.reset();
        bias_z_ctrl.reset();
    }
};

#endif // ABSTRACTORIENTATIONFILTER_H
