#ifndef RPYORIENTATIONVIEW_H
#define RPYORIENTATIONVIEW_H

#include "views/IBaseView.h"
#include "core/IOrientationProvider.h"

class QCustomPlot;

struct RPYOrientationView : IOrientationView
{
    RPYOrientationView(QCustomPlot * plot);
    ~RPYOrientationView() override = default;
    void update(IOrientationProvider * pvd) override;
    void clear() override;

private:
    QCustomPlot * plot;
};

#endif // RPYORIENTATIONVIEW_H
