#ifndef NUMCALIBRATIONVIEW_H
#define NUMCALIBRATIONVIEW_H

#include "views/base/IBaseView.h"
#include "magncalibrator.h"

class QLineEdit;

/*!
 * @brief The numerical view of calibration process.
 */
struct NumCalibrationView : ICalibrationView
{
    /*!
     * @brief NumCalibrationView constructor.
     * @param bias_x_le X-axis bias LineEdit.
     * @param bias_y_le Y-axis bias LineEdit.
     * @param bias_z_le Z-axis bias LineEdit.
     * @param scale_x_le X-axis scale LineEdit.
     * @param scale_y_le Y-axis scale LineEdit.
     * @param scale_z_le Z-axis scale LineEdit.
     */
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
