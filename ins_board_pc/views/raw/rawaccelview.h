#ifndef RAWACCELVIEW_H
#define RAWACCELVIEW_H

#include "views/xdaxisplotview.h"
#include "packets.h"

class QCustomPlot;

struct RawAccelView : RawPlotView
{
    RawAccelView(QCustomPlot * plot);
    ~RawAccelView() override = default;
    void update(const ViewModel & pvd) override;
};

#endif // RAWACCELVIEW_H
