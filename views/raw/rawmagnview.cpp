#include "views/raw/rawmagnview.h"
#include "views/base/plotsetup.h"
#include "adapters/rawviewmodel.h"
#include "qcustomplot.h"

RawMagnView::RawMagnView(QCustomPlot * plot) : plot{ plot }
{
    plots::setup_3axis_plot(plot);

    plot->plotLayout()->insertRow(0);
    plot->plotLayout()->addElement(0, 0, new QCPTextElement(plot, "Magnetometer sensor data"));

    plot->graph(0)->setName("x");
    plot->graph(1)->setName("y");
    plot->graph(2)->setName("z");

    plot->xAxis->setRange(0, 100);
    plot->xAxis->setLabel("packet");

    plot->yAxis->setRange(-500, 500);
    plot->yAxis->setLabel("Magnetic field, uT");
}

void RawMagnView::update(const ViewModel & vm)
{
    plots::update_3axis_plot(plot, vm.packet.m);
}

void RawMagnView::clear()
{
    plots::clear_3axis_plot(plot);
}
