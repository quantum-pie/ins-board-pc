#include "xdaxisplot.h"

XDAxisPlot::XDAxisPlot(QCustomPlot *plot) : plot{ plot }
{
    plot->addGraph();
    plot->addGraph();
    plot->addGraph();

    plot->legend->setVisible(true);

    plot->graph(0)->setPen(QPen(Qt::blue));
    plot->graph(1)->setPen(QPen(Qt::red));
    plot->graph(2)->setPen(QPen(Qt::cyan));

    plot->setBackground(Qt::lightGray);
    plot->axisRect()->setBackground(Qt::black);
    plot->legend->setBrush(Qt::lightGray);
}

void XDAxisPlot::update_plot(const Vector3D & vec)
{
    utils::update_3axis_plot(plot, vec);
}

void XDAxisPlot::clear_plot()
{
    utils::clear_3axis_plot(plot);
}
