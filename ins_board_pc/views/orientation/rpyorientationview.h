#ifndef RPYORIENTATIONVIEW_H
#define RPYORIENTATIONVIEW_H

#include "views/xdaxisplot.h"
#include "views/IBaseView.h"
#include "core/IOrientationProvider.h"
#include "packets.h"

class QCustomPlot;

struct RPYOrientationView : IOrientationView, private XDAxisPlot
{
    RPYOrientationView(QCustomPlot * plot);
    ~RPYOrientationView() override = default;
    void update(const ViewModel & pvd) override;
    void clear() override;
};

#endif // RPYORIENTATIONVIEW_H
