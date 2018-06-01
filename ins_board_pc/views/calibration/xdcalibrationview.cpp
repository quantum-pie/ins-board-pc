#include "views/calibration/xdcalibrationview.h"

#include <QWidget>
#include <QGridLayout>

XDCalibrationView::XDCalibrationView(QWidget *dummy_plot, QGridLayout *container_layout)
    : XDAxisScatter{ dummy_plot, container_layout, "Calibrated measurements"}
{}

void XDCalibrationView::update(const ViewModel & vm)
{
    for(auto it = vm.meas_begin(); it != vm.meas_end(); ++it)
    {
        update_scatter(*it);
    }
}

void XDCalibrationView::clear()
{
    clear_scatter();
}
