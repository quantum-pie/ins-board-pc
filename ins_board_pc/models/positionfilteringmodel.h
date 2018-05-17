#ifndef POSITIONFILTERINGMODEL_H
#define POSITIONFILTERINGMODEL_H

#include "ellipsoid.h"
#include "eigenaux.h"
#include "basefilteringmodel.h"

template<typename PositionFilter>
struct PositionFilteringModel : BaseFilteringModel<PositionFilter>
{
    PositionFilteringModel()
        : BaseFilteringModel<PositionFilter>()
    {}

    Vector3D get_cartesian() const
    {
        return this->filtering_strategy->get_cartesian();
    }

    Ellipsoid get_ellipsoid() const
    {
        return this->filtering_strategy->get_ellipsoid();
    }

    Vector3D get_velocity() const
    {
        return this->filtering_strategy->get_velocity();
    }

    Vector3D get_acceleration() const
    {
        return this->filtering_strategy->get_acceleration();
    }
};

#endif // POSITIONFILTERINGMODEL_H
