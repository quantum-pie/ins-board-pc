#ifndef XDCALIBRATIONVIEW_H
#define XDCALIBRATIONVIEW_H

#include "views/base/xdaxisscatter.h"
#include "views/base/IBaseView.h"
#include "magncalibrator.h"

/*!
 * @brief The scatterplot calibration process view.
 */
struct XDCalibrationView : ICalibrationView, private XDAxisScatter
{
    /*!
     * @brief XDCalibrationView constructor.
     * @param dummy_plot preconstructed empty plot.
     * @param container_layout layout containing empty plot.
     */
    XDCalibrationView(QWidget *dummy_plot, QGridLayout *container_layout);

    ~XDCalibrationView() override = default;

    void update(const ViewModel & vm) override;
    void clear() override;
};

#endif // XDCALIBRATIONVIEW_H
