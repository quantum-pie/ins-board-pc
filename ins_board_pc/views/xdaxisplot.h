#ifndef XDAXISPLOTVIEW_H
#define XDAXISPLOTVIEW_H

#include "qcustomplot.h"
#include "eigenaux.h"
#include "utils.h"

struct XDAxisPlot
{
    XDAxisPlot(QCustomPlot * plot);

    void update_plot(const Vector3D & vec);
    void clear_plot();

private:
    QCustomPlot * plot;
};

#endif // XDAXISPLOTVIEW_H
