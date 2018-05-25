#ifndef FILTERINGCONTROLLERSFWD_H
#define FILTERINGCONTROLLERSFWD_H

#include "controllers/direct/filtering/filteringcontroller.h"
#include "views/IBaseView.h"
#include "filtering/public_interfaces/IOrientationFilter.h"
#include "filtering/public_interfaces/IPositionFilter.h"

using PositionFilteringController = FilteringController<IPositionFilter, IPositionView>;
using OrientationFilteringController = FilteringController<IOrientationFilter, IOrientationView>;

#endif // FILTERINGCONTROLLERSFWD_H
