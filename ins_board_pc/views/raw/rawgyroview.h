#ifndef RAWGYROVIEW_H
#define RAWGYROVIEW_H

#include "views/xdaxisplotview.h"
#include "packets.h"

class QCustomPlot;

struct RawGyroView : RawPlotView
{
    RawGyroView(QCustomPlot * plot);
    ~RawGyroView() override = default;
    void update(const ViewModel & pvd) override;
};

#endif // RAWGYROVIEW_H
