#ifndef PLOTSETUP_H
#define PLOTSETUP_H

#include "eigenaux.h"

class QCustomPlot;
class QCPCurve;

namespace plots
{

void setup_rpy_plot(QCustomPlot * plot);
void setup_enu_plot(QCustomPlot * plot, QCPCurve * &raw_track, QCPCurve * &smoothed_track);
void setup_3axis_plot(QCustomPlot * plot);
void update_3axis_plot(QCustomPlot * plot, const Vector3D & vec);
void clear_3axis_plot(QCustomPlot * plot);
void update_track(QCustomPlot * plot, QCPCurve * track, const Vector3D & point);

}

#endif // PLOTSETUP_H
