#ifndef XDAXISSCATTER_H
#define XDAXISSCATTER_H

#include "eigenaux.h"

#include <QtDataVisualization>

class QWidget;
class QGridLayout;

using namespace QtDataVisualization;

struct XDAxisScatter
{
    XDAxisScatter(QWidget *dummy_plot, QGridLayout *container_layout, QString title);

    void update_scatter(const Vector3D & p);
    void clear_scatter();

private:
    Q3DScatter plot;
    QScatterDataArray data;
};

#endif // CALIBRATIONVIEW_H
