#ifndef NUMCALIBRATIONVIEW_H
#define NUMCALIBRATIONVIEW_H

#include "views/IBaseView.h"
#include "magncalibrator.h"

class QLineEdit;

struct NumCalibrationView : ICalibrationView
{
    NumCalibrationView(QLineEdit * bias_x_le, QLineEdit * bias_y_le, QLineEdit * bias_z_le,
                       QLineEdit * scale_x_le, QLineEdit * scale_y_le, QLineEdit * scale_z_le);

    ~NumCalibrationView() override = default;

    void update(const ViewModel & vm) override;
    void clear() override;

private:
    QLineEdit * bias_x_le;
    QLineEdit * bias_y_le;
    QLineEdit * bias_z_le;
    QLineEdit * scale_x_le;
    QLineEdit * scale_y_le;
    QLineEdit * scale_z_le;
};

#endif // NUMCALIBRATIONVIEW_H
