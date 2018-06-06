#ifndef RAWMAGNVIEW_H
#define RAWMAGNVIEW_H

#include "views/xdaxisplot.h"
#include "views/IBaseView.h"
#include "packets.h"

class QCustomPlot;

struct RawMagnView : IRawView, private XDAxisPlot
{
    RawMagnView(QCustomPlot * plot);
    ~RawMagnView() override = default;

    void update(const ViewModel & pvd) override;
    void clear();
};

#endif // RAWMAGNVIEW_H
