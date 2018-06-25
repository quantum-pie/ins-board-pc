#ifndef POSITIONFILTERINGVIEWMODEL_H
#define POSITIONFILTERINGVIEWMODEL_H

#include "adapters/adapter.h"
#include "eigenaux.h"
#include "filtering/public_interfaces/IPositionFilter.h"
#include "packets.h"
#include "qualitycontrol.h"

class Ellipsoid;

struct PositionFilteringViewModel
{
    Ellipsoid ellip;
    Vector3D pos;
    double ground_speed;
    double track_angle;
};

template<>
struct Adapter<IPositionFilter, PositionFilteringViewModel>
{
    PositionFilteringViewModel operator()(const IPositionFilter & filter);
    void set_accumulator_capacity(std::size_t new_capacity);

private:
    QualityControl<Vector3D> speed_accum;
};

template<>
struct Adapter<FilteredPacket, PositionFilteringViewModel>
{
    PositionFilteringViewModel operator()(const FilteredPacket & packet);
    void set_accumulator_capacity(std::size_t);
};

#endif // POSFILTERINGVIEWMODEL_H
