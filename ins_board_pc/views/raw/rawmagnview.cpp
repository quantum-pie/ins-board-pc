#include "views/raw/rawmagnview.h"
#include "qcustomplot.h"

RawMagnView::RawMagnView(QCustomPlot * plot) : XDAxisPlot{ plot }
{
    plot->plotLayout()->insertRow(0);
    plot->plotLayout()->addElement(0, 0, new QCPTextElement(plot, "Magnetometer sensor data"));

    plot->graph(0)->setName("x");
    plot->graph(1)->setName("y");
    plot->graph(2)->setName("z");

    plot->xAxis->setRange(0, 200);
    plot->xAxis->setLabel("packet");

    plot->yAxis->setRange(-500, 500);
    plot->yAxis->setLabel("Magnetic field, uT");
}

void RawMagnView::update(const ViewModel & vm)
{
    update_plot(vm.m);
}

void RawMagnView::clear()
{
    clear_plot();
}
