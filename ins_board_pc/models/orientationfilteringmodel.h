#ifndef ORIENTATIONFILTERINGMODEL_H
#define ORIENTATIONFILTERINGMODEL_H

#include "models/filteringmodel.h"
#include "core/IOrientationProviderCore.h"
#include "filtering/public_interfaces/IOrientationFilter.h"

class OrientationFilteringModel : public FilteringModel, IOrientationProviderCore
{
public:
    explicit OrientationFilteringModel(IOrientationFilter * wrapee)
        : FilteringModel{ wrapee }, wrapee{ wrapee } {}

    ~OrientationFilteringModel() override = default;

    quat::Quaternion get_orientation_quaternion() const
    {
        return do_get_orientation_quaternion();
    }

    Vector3D get_gyro_bias() const
    {
        return do_get_gyro_bias();
    }

private:
    quat::Quaternion do_get_orientation_quaternion() const
    {
        return wrapee->get_orientation_quaternion();
    }

    Vector3D do_get_gyro_bias() const
    {
        return wrapee->get_gyro_bias();
    }

    IOrientationFilter * wrapee;
};

#endif // ORIENTATIONFILTERINGMODEL_H
