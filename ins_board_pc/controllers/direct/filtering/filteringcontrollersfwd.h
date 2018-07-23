#ifndef FILTERINGCONTROLLERSFWD_H
#define FILTERINGCONTROLLERSFWD_H

#include "controllers/direct/filtering/filteringcontroller.h"
#include "views/base/IBaseView.h"
#include "filtering/public_interfaces/IOrientationFilter.h"
#include "filtering/public_interfaces/IPositionFilter.h"

//! Position filtering controller alias.
using PositionFilteringController = FilteringController<IPositionFilter, IPositionView>;

//! Orientation filtering controller alias.
using OrientationFilteringController = FilteringController<IOrientationFilter, IOrientationView>;

#endif // FILTERINGCONTROLLERSFWD_H
