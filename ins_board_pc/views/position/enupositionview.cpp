#include "views/position/enupositionview.h"
#include "views/base/plotsetup.h"
#include "adapters/positionfilteringviewmodel.h"
#include "packets.h"
#include "geometry.h"
#include "utils.h"

ENUPositionView::ENUPositionView(QCustomPlot *plot)
    : is_initialized{ false }, plot{ plot }
{
    plots::setup_enu_plot(plot, raw_track, smoothed_track);
}

void ENUPositionView::update(const IPositionView::ViewModel & vm)
{
    if(is_initialized)
    {
        Vector3D enu_flt = geom::ecef_to_enu(vm.pos - start_ecef, start_geo);
        plots::update_track(plot, smoothed_track, enu_flt);
    }
    else
    {
        is_initialized = true;
        start_geo = geom::cartesian_to_geodetic(vm.pos, vm.ellip);
        start_ecef = vm.pos;
    }
}

void ENUPositionView::update(const IRawView::ViewModel & vm)
{
    if(is_initialized)
    {
        Vector3D enu_raw = geom::ecef_to_enu(vm.gps_data.pos - start_ecef, start_geo);
        plots::update_track(plot, raw_track, enu_raw);
    }
    else
    {
        is_initialized = true;
        start_geo = vm.gps_data.geo;
        start_ecef = vm.gps_data.pos;
    }
}

void ENUPositionView::clear()
{
    is_initialized = false;
    raw_track->setData({}, {});
    smoothed_track->setData({}, {});
}


