#include "views/raw/rawaccelview.h"
#include "qcustomplot.h"

RawAccelView::RawAccelView(QCustomPlot * plot) : XDAxisPlot{ plot }
{
    plot->plotLayout()->insertRow(0);
    plot->plotLayout()->addElement(0, 0, new QCPTextElement(plot, "Acceleration sensor data"));

    plot->graph(0)->setName("x");
    plot->graph(1)->setName("y");
    plot->graph(2)->setName("z");

    plot->xAxis->setRange(0, 200);
    plot->xAxis->setLabel("packet");

    plot->yAxis->setRange(-2000, 2080);
    plot->yAxis->setLabel("Acceleration, mg");
}

void RawAccelView::update(const ViewModel & vm)
{
    update_plot(vm.a * 1e3);
}

void RawAccelView::clear()
{
    clear_plot();
}
