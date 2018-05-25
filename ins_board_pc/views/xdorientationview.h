#ifndef XDORIENTATIONVIEW_H
#define XDORIENTATIONVIEW_H

#include "views/IBaseView.h"
#include "core/IOrientationProvider.h"

struct XDOrientationView : IOrientationView
{
    void update(IOrientationProvider * pvd) override;
};

#endif // XDORIENTATIONVIEW_H
