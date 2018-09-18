#include "views/raw/rawgyroview.h"
#include "views/base/plotsetup.h"
#include "adapters/rawviewmodel.h"
#include "qcustomplot.h"

RawGyroView::RawGyroView(QCustomPlot * plot) : plot{ plot }
{
    plots::setup_3axis_plot(plot);

    plot->plotLayout()->insertRow(0);
    plot->plotLayout()->addElement(0, 0, new QCPTextElement(plot, "Gyroscope sensor data"));

    plot->graph(0)->setName("x");
    plot->graph(1)->setName("y");
    plot->graph(2)->setName("z");

    plot->xAxis->setRange(0, 100);
    plot->xAxis->setLabel("packet");

    plot->yAxis->setRange(-250, 250);
    plot->yAxis->setLabel("Angular rate, dps");
}

void RawGyroView::update(const ViewModel & vm)
{
    plots::update_3axis_plot(plot, vm.packet.w);
}

void RawGyroView::clear()
{
    plots::clear_3axis_plot(plot);
}
