#ifndef XDORIENTATIONVIEW_H
#define XDORIENTATIONVIEW_H

#include "models/orientationfilteringmodel.h"

template<typename OrientationFilter>
class XDOrientationView
{
public:
    using model_type = OrientationFilteringModel<OrientationFilter>;
    XDOrientationView(const model_type & model);

private:
    model_type & model;
}

#endif // XDORIENTATIONVIEW_H
