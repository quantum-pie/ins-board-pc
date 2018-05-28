#include "views/stdorientationview.h"
#include "utils.h"

#include <QLineEdit>

StdOrientationView::StdOrientationView(QLineEdit *roll_std_le, QLineEdit *pitch_std_le, QLineEdit *yaw_std_le)
    : roll_std_le{ roll_std_le }, pitch_std_le{ pitch_std_le }, yaw_std_le{ yaw_std_le }
{}

void StdOrientationView::set_accumulator_capacity(std::size_t new_capacity)
{
    rpy_ctrl.set_sampling(new_capacity);
}

void StdOrientationView::update(IOrientationProvider *pvd)
{
    rpy_ctrl.update(pvd->get_orientation_quaternion().rpy());
    if(rpy_ctrl.is_saturated())
    {
        auto rpy_std = rpy_ctrl.get_std().unaryExpr(&utils::radians_to_degrees);
        roll_std_le->setText(utils::double_view(rpy_std[0]));
        pitch_std_le->setText(utils::double_view(rpy_std[1]));
        yaw_std_le->setText(utils::double_view(rpy_std[2]));
    }
}

void StdOrientationView::clear()
{

}
