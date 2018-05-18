#include "complorientationfilteringmodel.h"

void ComplOrientationFilteringModel::set_static_accel_gain(double gain)
{
    filtering_strategy->set_static_accel_gain(gain);
}

void ComplOrientationFilteringModel::set_static_magn_gain(double gain)
{
    filtering_strategy->set_static_magn_gain(gain);
}

void ComplOrientationFilteringModel::set_bias_gain(double gain)
{
    filtering_strategy->set_bias_gain(gain);
}

double ComplOrientationFilteringModel::get_static_accel_gain() const
{
    return filtering_strategy->get_static_accel_gain();
}

double ComplOrientationFilteringModel::get_static_magn_gain() const
{
    return filtering_strategy->get_static_magn_gain();
}

double ComplOrientationFilteringModel::get_bias_gain() const
{
    return filtering_strategy->get_bias_gain();
}
