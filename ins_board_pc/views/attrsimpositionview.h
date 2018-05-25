#ifndef ATTRSIMPOSITIONVIEW_H
#define ATTRSIMPOSITIONVIEW_H

#include "core/ISimPositionAttr.h".h"
#include "views/IBaseView.h"

struct AttrKalmanPositionView : IBaseView<ISimPositionAttr>
{
    void update(ISimPositionAttr * pvd) override;
};

#endif // ATTRSIMPOSITIONVIEW_H
