#ifndef XDAXISPLOTVIEW_H
#define XDAXISPLOTVIEW_H

#include "views/IBaseView.h"
#include "qcustomplot.h"
#include "eigenaux.h"
#include "utils.h"

template<typename ViewModel>
struct XDAxisPlotView : IBaseView<ViewModel>
{
    XDAxisPlotView(QCustomPlot * plot) : plot{ plot }
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

    ~XDAxisPlotView() override = default;

    void update_plot(const Vector3D & vec)
    {
        utils::update_3axis_plot(plot, vec);
    }

    void clear() override
    {
        utils::clear_3axis_plot(plot);
    }

private:
    QCustomPlot * plot;
};

using RawPlotView = XDAxisPlotView<RawPacket>;
using OrientationPlotView = XDAxisPlotView<OrientationFilteringViewModel>;

#endif // XDAXISPLOTVIEW_H
