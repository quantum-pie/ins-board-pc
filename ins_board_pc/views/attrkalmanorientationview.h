#ifndef ATTRKALMANORIENTATIONVIEW_H
#define ATTRKALMANORIENTATIONVIEW_H

#include "core/IKalmanOrientationAttr.h"
#include "views/IBaseView.h"

struct AttrKalmanOrientationView : IBaseView<IKalmanOrientationAttr>
{
    void update(IKalmanOrientationAttr * pvd) override;
};

#endif // ATTRKALMANORIENTATIONVIEW_H
