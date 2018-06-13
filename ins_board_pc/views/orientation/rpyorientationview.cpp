#include "views/orientation/rpyorientationview.h"
#include "qcustomplot.h"
#include "utils.h"

RPYOrientationView::RPYOrientationView(QCustomPlot * plot) : XDAxisPlot{ plot }
{
    plot->plotLayout()->insertRow(0);
    plot->plotLayout()->addElement(0, 0, new QCPTextElement(plot, "Roll Pitch Yaw"));

    plot->graph(0)->setName("R");
    plot->graph(1)->setName("P");
    plot->graph(2)->setName("Y");

    plot->xAxis->setRange(0, 200);
    plot->xAxis->setLabel("packet");

    plot->yAxis->setRange(-180, 180);
    plot->yAxis->setLabel("Angle, deg");
}

void RPYOrientationView::update(const ViewModel & vm)
{
    Vector3D rpy = vm.get_orientation_quaternion().rpy().unaryExpr(&utils::radians_to_degrees);
    update_plot(rpy);
}

void RPYOrientationView::clear()
{
    clear_plot();
}
