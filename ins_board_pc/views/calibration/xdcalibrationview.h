#ifndef XDCALIBRATIONVIEW_H
#define XDCALIBRATIONVIEW_H

#include "views/xdaxisscatter.h"
#include "views/IBaseView.h"
#include "magncalibrator.h"

struct XDCalibrationView : ICalibrationView, private XDAxisScatter
{
    XDCalibrationView(QWidget *dummy_plot, QGridLayout *container_layout);
    ~XDCalibrationView() override = default;

    void update(const ViewModel & vm) override;
    void clear() override;
};

#endif // XDCALIBRATIONVIEW_H
