#include "views/enupositionview.h"
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

void ENUPositionView::update(IPositionProvider *pvd)
{
    if(is_initialized)
    {
        Vector3D enu = geom::ecef_to_enu(pvd->get_cartesian(), start_geo);
        NumVector enu = curr_pf->get_position_enu();

        NumVector raw_pos(3);
        raw_pos << in.gps.x, in.gps.y, in.gps.z;
        NumVector enu_raw = curr_pf->get_position_enu(raw_pos);

        kalman_smooth_track->addData(enu[0], enu[1]);
        kalman_raw_track->addData(enu_raw[0], enu_raw[1]);

        update_enu_plot(ui->plot5);
    }
    else
    {
        is_initialized = true;
        start_geo = geom::cartesian_to_geodetic(pvd->get_cartesian(), pvd->get_ellipsoid());
    }

}

void ENUPositionView::clear()
{
    is_initialized = false;
    raw_track.data().clear();
    smoothed_track.data().clear();
}


