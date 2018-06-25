#include "views/raw/rawaccelview.h"
#include "views/base/plotsetup.h"
#include "qcustomplot.h"

RawAccelView::RawAccelView(QCustomPlot * plot) : plot{ plot }
{
    plots::setup_3axis_plot(plot);

    plot->plotLayout()->insertRow(0);
    plot->plotLayout()->addElement(0, 0, new QCPTextElement(plot, "Acceleration sensor data"));

    plot->graph(0)->setName("x");
    plot->graph(1)->setName("y");
    plot->graph(2)->setName("z");

    plot->xAxis->setRange(0, 100);
    plot->xAxis->setLabel("packet");

    plot->yAxis->setRange(-2000, 2080);
    plot->yAxis->setLabel("Acceleration, mg");
}

void RawAccelView::update(const ViewModel & vm)
{
    plots::update_3axis_plot(plot, vm.a * 1e3);
}

void RawAccelView::clear()
{
    plots::clear_3axis_plot(plot);
}
