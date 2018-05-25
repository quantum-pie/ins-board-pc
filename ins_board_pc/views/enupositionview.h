#ifndef ENUPOSITIONVIEW_H
#define ENUPOSITIONVIEW_H

#include "core/IPositionProvider.h"
#include "views/IBaseView.h"

struct ENUPositionView : IPositionView
{
    void update(IPositionProvider * pvd) override;
};

#endif // ENUPOSITIONVIEW_H
