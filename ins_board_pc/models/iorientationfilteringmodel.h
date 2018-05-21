#ifndef IORIENTATIONFILTERINGMODEL_H
#define IORIENTATIONFILTERINGMODEL_H

#include "models/IFilteringModel.h"
#include "core/IOrientationProviderCore.h"
#include "quaternion.h"

struct IOrientationFilteringModel : IFilteringModel, private IOrientationProviderCore
{
    quat::Quaternion get_orientation_quaternion() const
    {
        return do_get_orientation_quaternion();
    }

    Vector3D get_gyro_bias() const
    {
        return do_get_gyro_bias();
    }

    ~IOrientationFilter() override = default;
};

#endif // IORIENTATIONFILTERINGMODEL_H
