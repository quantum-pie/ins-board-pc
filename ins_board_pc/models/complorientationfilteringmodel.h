#ifndef COMPLORIENTATIONFILTERINGMODEL_H
#define COMPLORIENTATIONFILTERINGMODEL_H

#include "models/orientationfilteringmodel.h"
#include "filtering/public_interfaces/IComplementOrientationFilter.h"

struct ComplOrientationFilteringModel : OrientationFilteringModel<IComplementOrientationFilter>
{
    void set_static_accel_gain(double gain);
    void set_static_magn_gain(double gain);
    void set_bias_gain(double gain);

    double get_static_accel_gain() const;
    double get_static_magn_gain() const;
    double get_bias_gain() const;
};

#endif // COMPLORIENTATIONFILTERINGMODEL_H
