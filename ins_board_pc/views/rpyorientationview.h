#ifndef RPYORIENTATIONVIEW_H
#define RPYORIENTATIONVIEW_H

#include "views/IBaseView.h"
#include "core/IOrientationProvider.h"

struct RPYOrientationView : IOrientationView
{
    void update(IOrientationProvider * pvd) override
    {

    }
};

#endif // RPYORIENTATIONVIEW_H
