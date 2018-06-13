#include "views/position/trackpositionview.h"
#include "geometry.h"
#include "utils.h"

#include <QLineEdit>

TrackPositionView::TrackPositionView(QLineEdit *track_angle_le, QLineEdit *ground_speed_le)
    : track_angle_le{ track_angle_le}, ground_speed_le{ ground_speed_le }
{
    track_angle_le->setReadOnly(true);
    ground_speed_le->setReadOnly(true);
}

void TrackPositionView::set_accumulator_capacity(std::size_t new_capacity)
{
    speed_accum.set_sampling(new_capacity);
}

void TrackPositionView::update(const ViewModel & vm)
{
    speed_accum.update(vm.get_velocity());
    if(speed_accum.is_saturated())
    {
        Vector3D geo = geom::cartesian_to_geodetic(vm.get_cartesian(), vm.get_ellipsoid());
        Vector3D avg_speed = geom::ecef_to_enu(speed_accum.get_mean(), geo);

        double track_angle = utils::radians_to_degrees(geom::track_angle(avg_speed, geo));
        double ground_speed = utils::ms_to_knots(geom::ground_speed(avg_speed, geo));

        track_angle_le->setText(utils::double_view(track_angle));
        ground_speed_le->setText(utils::double_view(ground_speed));
    }
}

void TrackPositionView::clear()
{

}
