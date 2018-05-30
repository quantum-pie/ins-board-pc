#ifndef RPYORIENTATIONVIEW_H
#define RPYORIENTATIONVIEW_H

#include "views/xdaxisplotview.h"
#include "core/IOrientationProvider.h"
#include "packets.h"

class QCustomPlot;

struct RPYOrientationView : OrientationPlotView
{
    RPYOrientationView(QCustomPlot * plot);
    ~RPYOrientationView() override = default;
    void update(const ViewModel & pvd) override;
};

#endif // RPYORIENTATIONVIEW_H
