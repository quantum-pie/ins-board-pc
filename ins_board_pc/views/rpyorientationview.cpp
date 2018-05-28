#include "views/rpyorientationview.h"
#include "qcustomplot.h"
#include "utils.h"

RPYOrientationView::RPYOrientationView(QCustomPlot * plot) : plot{ plot }
{
    plot->plotLayout()->insertRow(0);
    plot->plotLayout()->addElement(0, 0, new QCPTextElement(plot, "Roll Pitch Yaw"));

    plot->addGraph();
    plot->addGraph();
    plot->addGraph();

    plot->graph(0)->setName("R");
    plot->graph(1)->setName("P");
    plot->graph(2)->setName("Y");

    plot->legend->setVisible(true);

    plot->graph(0)->setPen(QPen(Qt::blue));
    plot->graph(1)->setPen(QPen(Qt::red));
    plot->graph(2)->setPen(QPen(Qt::cyan));

    plot->xAxis->setRange(0, 200);
    plot->xAxis->setLabel("packet");

    plot->yAxis->setRange(-180, 180);
    plot->yAxis->setLabel("Angle, deg");

    plot->setBackground(Qt::lightGray);
    plot->axisRect()->setBackground(Qt::black);
    plot->legend->setBrush(Qt::lightGray);
}

void RPYOrientationView::update(IOrientationProvider *pvd)
{
    Vector3D rpy = pvd->get_orientation_quaternion().rpy();
    utils::update_3axis_plot(plot, rpy.unaryExpr(&utils::radians_to_degrees));
}

void RPYOrientationView::clear()
{
    utils::clear_3axis_plot(plot);
}
