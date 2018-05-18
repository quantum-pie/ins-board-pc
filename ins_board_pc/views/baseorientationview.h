#ifndef BASEORIENTATIONVIEW_H
#define BASEORIENTATIONVIEW_H

#include "models/orientationfilteringmodel.h"

template<typename OrientationFilter>
class BaseOrientationView
{
public:
    using model_type = OrientationFilteringModel<OrientationFilter>;

    BaseOrientationView(const model_type & model)
        : model{ model } {}



protected:
    model_type & model;
}


#endif // BASEORIENTATIONVIEW_H
