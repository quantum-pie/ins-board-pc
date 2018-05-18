#ifndef SIMPOSITIONFILTERINGMODEL_H
#define SIMPOSITIONFILTERINGMODEL_H

#include "models/positionfilteringmodel.h"
#include "filtering/filters/positionsim.h"

struct SimPositionFilteringModel : PositionFilteringModel<PositionSim>
{
    void set_initial_track(double radians);
    void set_speed(double ms);

    double get_initial_track() const;
    double get_speed() const;
};

#endif // SIMPOSITIONFILTERINGMODEL_H
