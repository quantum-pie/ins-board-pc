#ifndef RAWGYROVIEW_H
#define RAWGYROVIEW_H

#include "views/xdaxisplot.h"
#include "views/IBaseView.h"
#include "packets.h"

class QCustomPlot;

struct RawGyroView : IRawView, private XDAxisPlot
{
    RawGyroView(QCustomPlot * plot);
    ~RawGyroView() override = default;

    void update(const ViewModel & pvd) override;
    void clear() override;
};

#endif // RAWGYROVIEW_H
