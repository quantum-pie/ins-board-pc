#include "views/enupositionview.h"
#include "packets.h"
#include "geometry.h"

ENUPositionView::ENUPositionView(QCustomPlot *plot)
    : is_initialized{ false }, plot{ plot },
      raw_track{ plot->xAxis, plot->yAxis },
      smoothed_track{ plot->xAxis, plot->yAxis }
{
    plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    plot->plotLayout()->insertRow(0);
    plot->plotLayout()->addElement(0, 0, new QCPTextElement(plot, "ENU relative position"));

    raw_track.setName("Raw track");
    raw_track.setPen(QPen(Qt::blue));

    smoothed_track.setName("Smoothed track");
    smoothed_track.setPen(QPen(Qt::red));

    plot->legend->setVisible(true);

    plot->xAxis->setRange(-10, 10);
    plot->xAxis->setLabel("E, m");

    plot->yAxis->setRange(-10, 10);
    plot->yAxis->setLabel("N, m");

    plot->setBackground(Qt::lightGray);
    plot->axisRect()->setBackground(Qt::black);
    plot->legend->setBrush(Qt::lightGray);
}

void ENUPositionView::update(const ViewModel & vm)
{
    if(is_initialized)
    {
        Vector3D enu_flt = geom::ecef_to_enu(vm.pvd_ref.get_cartesian(), start_geo);
        Vector3D enu_raw = geom::ecef_to_enu(vm.raw_ref.pos, start_geo);

        smoothed_track.addData(enu_flt[0], enu_flt[1]);
        raw_track.addData(enu_raw[0], enu_raw[1]);

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
    else
    {
        is_initialized = true;
        start_geo = geom::cartesian_to_geodetic(vm.pvd_ref.get_cartesian(), vm.pvd_ref.get_ellipsoid());
    }

}

void ENUPositionView::clear()
{
    is_initialized = false;
    raw_track.data().clear();
    smoothed_track.data().clear();
}


