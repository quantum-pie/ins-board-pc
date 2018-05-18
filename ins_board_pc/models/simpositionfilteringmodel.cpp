#include "models/simpositionfilteringmodel.h"

void SimPositionFilteringModel::set_initial_track(double radians)
{
    filtering_strategy->set_initial_track(radians);
}

void SimPositionFilteringModel::set_speed(double ms)
{
    filtering_strategy->set_speed(ms);
}

double SimPositionFilteringModel::get_initial_track() const
{
    return filtering_strategy->get_initial_track();
}

double SimPositionFilteringModel::get_speed() const
{
    return filtering_strategy->get_speed();
}
