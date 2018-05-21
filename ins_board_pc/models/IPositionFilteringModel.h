#ifndef IPOSITIONFILTERINGMODEL_H
#define IPOSITIONFILTERINGMODEL_H

#include "models/IFilteringModel.h"
#include "core/ipositionprovidercore.h"

#include "eigenaux.h"
#include "ellipsoid.h"

struct IPositionFilteringModel : IFilteringModel, private IPositionProviderCore
{
    Vector3D get_cartesian() const
    {
        return do_get_cartesian();
    }

    Ellipsoid get_ellipsoid() const
    {
        return do_get_ellipsoid();
    }

    Vector3D get_velocity() const
    {
        return do_get_velocity();
    }

    Vector3D get_acceleration() const
    {
        return do_get_acceleration();
    }

    ~IPositionFilteringModel() override = default;
};

#endif // IPOSITIONFILTERINGMODEL_H
