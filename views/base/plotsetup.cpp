#include "views/base/plotsetup.h"
#include "qcustomplot.h"

namespace plots
{

void setup_rpy_plot(QCustomPlot * plot)
{
    setup_3axis_plot(plot);

    plot->plotLayout()->insertRow(0);
    plot->plotLayout()->addElement(0, 0, new QCPTextElement(plot, "Roll Pitch Yaw"));

    plot->graph(0)->setName("R");
    plot->graph(1)->setName("P");
    plot->graph(2)->setName("Y");

    plot->xAxis->setRange(0, 100);
    plot->xAxis->setLabel("packet");

    plot->yAxis->setRange(-180, 180);
    plot->yAxis->setLabel("Angle, deg");
}

void setup_enu_plot(QCustomPlot * plot, QCPCurve * &raw_track, QCPCurve * &smoothed_track)
{
    plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    plot->plotLayout()->insertRow(0);
    plot->plotLayout()->addElement(0, 0, new QCPTextElement(plot, "ENU relative position"));

    raw_track = new QCPCurve(plot->xAxis, plot->yAxis);
    smoothed_track = new QCPCurve(plot->xAxis, plot->yAxis);

    raw_track->setName("Raw track");
    raw_track->setPen(QPen(Qt::blue));

    smoothed_track->setName("Smoothed track");
    smoothed_track->setPen(QPen(Qt::red));

    plot->legend->setVisible(true);

    plot->xAxis->setRange(-10, 10);
    plot->xAxis->setLabel("E, m");

    plot->yAxis->setRange(-10, 10);
    plot->yAxis->setLabel("N, m");

    plot->setBackground(Qt::lightGray);
    plot->axisRect()->setBackground(Qt::black);
    plot->legend->setBrush(Qt::lightGray);
}

void setup_3axis_plot(QCustomPlot * plot)
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

void update_3axis_plot(QCustomPlot * plot, const Vector3D & vec)
{
    int pts = plot->graph(0)->dataCount();
    if(pts < plot->xAxis->range().upper)
    {
        plot->graph(0)->addData(pts, vec[0]);
        plot->graph(1)->addData(pts, vec[1]);
        plot->graph(2)->addData(pts, vec[2]);
        plot->replot();
    }
    else
    {
        plot->graph(0)->data()->clear();
        plot->graph(1)->data()->clear();
        plot->graph(2)->data()->clear();
    }
}

void clear_3axis_plot(QCustomPlot * plot)
{
    plot->graph(0)->data().clear();
    plot->graph(1)->data().clear();
    plot->graph(2)->data().clear();
}

void update_track(QCustomPlot * plot, QCPCurve * track, const Vector3D & point, int track_len)
{
    track->addData(point[0], point[1]);
    auto track_data = track->data();

    if(track_data->size() > track_len)
    {
        track_data->remove(track_data->begin()->sortKey());
    }

    plot->rescaleAxes();

    double expected_y_span = plot->xAxis->range().size() * plot->axisRect()->height() / plot->axisRect()->width();

    if(plot->yAxis->range().size() < expected_y_span)
    {
        plot->yAxis->setScaleRatio(plot->xAxis, 1);
    }
    else
    {
        plot->xAxis->setScaleRatio(plot->yAxis, 1);
    }

    plot->replot();
}

}
