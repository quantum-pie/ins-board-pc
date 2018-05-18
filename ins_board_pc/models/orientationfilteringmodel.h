#ifndef ORIENTATIONFILTERINGMODEL_H
#define ORIENTATIONFILTERINGMODEL_H

#include "quaternion.h"
#include "eigenaux.h"
#include "basefilteringmodel.h"

template<typename OrientationFilter>
struct OrientationFilteringModel : BaseFilteringModel<OrientationFilter>
{
    OrientationFilteringModel()
        : BaseFilteringModel<OrientationFilter>()
    {}

    quat::Quaternion get_orientation_quaternion() const
    {
        return this->filtering_strategy->get_orientation_quaternion();
    }

    Vector3D get_gyro_bias() const
    {
        return this->filtering_strategy->get_gyro_bias();
    }
};

#endif // ORIENTATIONFILTERINGMODEL_H
