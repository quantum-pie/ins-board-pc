#include "magncalibrationcontroller.h"
#include "receiver.h"

#include <QPushButton>

MagnCalibrationController::MagnCalibrationController(MagnCalibrator & calibrator, const QPushButton *calibrate_btn, const QPushButton *save_btn)
    : RunningFlag{ calibrate_btn->isChecked() }, calibration_model{ calibrator }
{
    connect(calibrate_btn, SIGNAL(toggled(bool)), this, SLOT(handle_calibrate(bool)));
    connect(save_btn, SIGNAL(clicked(bool)), this, SLOT(save_calibration()));
}

void MagnCalibrationController::handle_input(const RawPacket & z)
{
    if(is_running())
    {
        calibration_model.get().update(z.m);
    }
}

void MagnCalibrationController::handle_calibrate(bool en)
{
    set_running(en);
    if(en)
    {
        calibration_model.get().reset();
        clear_views();
    }
    else
    {
        calibration_model.get().fit();
        update_views(calibration_model);
    }

    calibration_model.get().clear_meas();
}

void MagnCalibrationController::save_calibration()
{
    calibration_model.get().save();
}
