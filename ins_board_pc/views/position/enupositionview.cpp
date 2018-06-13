#include "views/position/enupositionview.h"
#include "packets.h"
#include "geometry.h"
#include "utils.h"

ENUPositionView::ENUPositionView(QCustomPlot *plot)
    : is_initialized{ false }, plot{ plot }
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

void ENUPositionView::update(const IPositionView::ViewModel & vm)
{
    if(is_initialized)
    {
        Vector3D enu_flt = geom::ecef_to_enu(vm.get_cartesian(), start_geo);
        utils::update_track(plot, smoothed_track, enu_flt);
    }
    else
    {
        is_initialized = true;
        start_geo = geom::cartesian_to_geodetic(vm.get_cartesian(), vm.get_ellipsoid());
    }

}

void ENUPositionView::update(const IRawView::ViewModel & vm)
{
    if(is_initialized)
    {
        Vector3D enu_raw = geom::ecef_to_enu(vm.gps_data.pos, start_geo);
        utils::update_track(plot, raw_track, enu_raw);
    }
    else
    {
        is_initialized = true;
        start_geo = vm.gps_data.geo;
    }

}

void ENUPositionView::clear()
{
    is_initialized = false;
    raw_track->data().clear();
    smoothed_track->data().clear();
}


