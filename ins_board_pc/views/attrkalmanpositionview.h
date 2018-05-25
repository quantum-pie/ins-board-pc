#ifndef ATTRKALMANPOSITIONVIEW_H
#define ATTRKALMANPOSITIONVIEW_H

#include "core/IKalmanPositionAttr.h"
#include "views/IBaseView.h"

struct AttrKalmanPositionView : IBaseView<IKalmanPositionAttr>
{
    void update(IKalmanPositionAttr * pvd) override;
};

#endif // ATTRKALMANPOSITIONVIEW_H
