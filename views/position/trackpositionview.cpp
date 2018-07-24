#include "views/position/trackpositionview.h"
#include "adapters/positionfilteringviewmodel.h"
#include "geometry.h"
#include "utils.h"

#include <QLineEdit>

TrackPositionView::TrackPositionView(QLineEdit *track_angle_le, QLineEdit *ground_speed_le)
    : track_angle_le{ track_angle_le}, ground_speed_le{ ground_speed_le }
{
    track_angle_le->setReadOnly(true);
    ground_speed_le->setReadOnly(true);
}

void TrackPositionView::update(const ViewModel & vm)
{
    double track_angle = utils::radians_to_degrees(vm.track_angle);
    double ground_speed = vm.ground_speed;

    track_angle_le->setText(utils::double_view(track_angle));
    ground_speed_le->setText(utils::double_view(ground_speed));
}

void TrackPositionView::clear()
{

}
