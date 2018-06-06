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
    QScatterDataArray * data;
    Q3DScatter plot;
};

#endif // CALIBRATIONVIEW_H
