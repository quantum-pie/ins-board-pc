#ifndef RAWACCELVIEW_H
#define RAWACCELVIEW_H

#include "views/xdaxisplot.h"
#include "views/IBaseView.h"
#include "packets.h"

class QCustomPlot;

struct RawAccelView : IRawView, private XDAxisPlot
{
    RawAccelView(QCustomPlot * plot);
    ~RawAccelView() override = default;
    void update(const ViewModel & pvd) override;
    void clear() override;
};

#endif // RAWACCELVIEW_H
