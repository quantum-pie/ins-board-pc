#ifndef POSITIONFILTERINGMODEL_H
#define POSITIONFILTERINGMODEL_H

#include "models/filteringmodel.h"
#include "core/IPositionProviderCore.h"
#include "filtering/public_interfaces/IPositionFilter.h"

class PositionFilteringModel : public FilteringModel, IPositionProviderCore
{
public:
    explicit PositionFilteringModel(IPositionFilter * wrapee)
        : FilteringModel{ wrapee }, wrappee{ wrapee } {}

    ~PositionFilteringModel() override = default;

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

private:
    Vector3D do_get_cartesian() const override
    {
        return wrapee->get_cartesian();
    }

    Ellipsoid do_get_ellipsoid() const override
    {
        return wrapee->get_ellipsoid();
    }

    Vector3D do_get_velocity() const override
    {
        return wrapee->get_velocity();
    }

    Vector3D do_get_acceleration() const override
    {
        return wrapee->get_acceleration();
    }

    IPositionFilter * wrapee;
};

#endif // POSITIONFILTERINGMODEL_H
