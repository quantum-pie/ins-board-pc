#ifndef RAWMAGNVIEW_H
#define RAWMAGNVIEW_H

#include "views/xdaxisplotview.h"
#include "packets.h"

class QCustomPlot;

struct RawMagnView : RawPlotView
{
    RawMagnView(QCustomPlot * plot);
    ~RawMagnView() override = default;
    void update(const ViewModel & pvd) override;
};

#endif // RAWMAGNVIEW_H
