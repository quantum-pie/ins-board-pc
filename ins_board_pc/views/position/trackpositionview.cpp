#include "views/position/trackpositionview.h"
#include "geometry.h"
#include "utils.h"

#include <QLineEdit>

TrackPositionView::TrackPositionView(QLineEdit * samples_le, QLineEdit *track_angle_le, QLineEdit *ground_speed_le)
    : track_angle_le{ track_angle_le}, ground_speed_le{ ground_speed_le }
{
    connect(samples_le, SIGNAL(textEdited(QString)), this, SLOT(set_accumulator_capacity(QString)));
}

void TrackPositionView::set_accumulator_capacity(const QString & str)
{
    speed_accum.set_sampling(str.toInt());
}

void TrackPositionView::update(const ViewModel & vm)
{
    speed_accum.update(vm.pvd_ref.get_velocity());
    if(speed_accum.is_saturated())
    {
        Vector3D geo = geom::cartesian_to_geodetic(vm.pvd_ref.get_cartesian(), vm.pvd_ref.get_ellipsoid());
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
