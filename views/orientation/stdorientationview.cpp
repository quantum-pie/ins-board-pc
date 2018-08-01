#include "views/orientation/stdorientationview.h"
#include "adapters/orientationfilteringviewmodel.h"
#include "utils.h"

#include <QLineEdit>

StdOrientationView::StdOrientationView(QLineEdit *roll_std_le, QLineEdit *pitch_std_le, QLineEdit *yaw_std_le, QLineEdit *roll_le, QLineEdit *pitch_le, QLineEdit * magnetic_heading_le)
    : roll_std_le{ roll_std_le }, pitch_std_le{ pitch_std_le }, yaw_std_le{ yaw_std_le }, roll_le{ roll_le }, pitch_le{ pitch_le }, magnetic_heading_le{ magnetic_heading_le }
{
    roll_std_le->setReadOnly(true);
    pitch_std_le->setReadOnly(true);
    yaw_std_le->setReadOnly(true);
    roll_le->setReadOnly(true);
    pitch_le->setReadOnly(true);
    magnetic_heading_le->setReadOnly(true);
}

void StdOrientationView::update(const ViewModel & vm)
{
    Vector3D rpy = vm.rpy.unaryExpr(&utils::radians_to_degrees);
    Vector3D rpy_std = vm.rpy_std.unaryExpr(&utils::radians_to_degrees);
    roll_std_le->setText(utils::double_view(rpy_std[0]));
    pitch_std_le->setText(utils::double_view(rpy_std[1]));
    yaw_std_le->setText(utils::double_view(rpy_std[2]));
    roll_le->setText(utils::double_view(rpy[0]));
    pitch_le->setText(utils::double_view(rpy[1]));
    magnetic_heading_le->setText(utils::double_view(utils::radians_to_degrees(vm.rpy_mean[2])));
}

void StdOrientationView::clear()
{

}
