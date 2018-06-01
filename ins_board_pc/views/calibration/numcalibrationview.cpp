#include "views/calibration/numcalibrationview.h"
#include "utils.h"

#include <QLineEdit>

NumCalibrationView::NumCalibrationView(QLineEdit *bias_x_le, QLineEdit *bias_y_le, QLineEdit *bias_z_le,
                                       QLineEdit *scale_x_le, QLineEdit *scale_y_le, QLineEdit *scale_z_le)
    : bias_x_le{ bias_x_le }, bias_y_le{ bias_y_le }, bias_z_le{ bias_z_le },
      scale_x_le{ scale_x_le }, scale_y_le{ scale_y_le }, scale_z_le{ scale_z_le }
{}

void NumCalibrationView::update(const ViewModel & vm)
{
    Vector3D bias = vm.get_bias();
    Matrix3D scale = vm.get_scale();

    bias_x_le->setText(utils::double_view(bias[0], 4));
    bias_y_le->setText(utils::double_view(bias[1], 4));
    bias_z_le->setText(utils::double_view(bias[2], 4));

    scale_x_le->setText(utils::double_view(scale(0, 0), 4));
    scale_x_le->setText(utils::double_view(scale(1, 1), 4));
    scale_z_le->setText(utils::double_view(scale(2, 2), 4));
}

void NumCalibrationView::clear()
{

}
