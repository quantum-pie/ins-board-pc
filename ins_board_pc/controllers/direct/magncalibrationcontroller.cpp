#include "magncalibrationcontroller.h"
#include "receiver.h"

#include <QPushButton>

MagnCalibrationController::MagnCalibrationController(MagnCalibrator & calibrator, const Receiver *receiver, const QPushButton *calibrate_btn)
    : RunningFlag{ calibrate_btn->isChecked() }, calibration_model{ calibrator }
{
    connect(receiver, SIGNAL(raw_packet_received(RawPacket)), this, SLOT(handle_input(RawPacket)));
    connect(calibrate_btn, SIGNAL(toggled(bool)), this, SLOT(handle_calibrate(bool)));
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
        clear_views();
    }
    else
    {
        calibration_model.get().fit();
        update_views(calibration_model);
    }

    calibration_model.get().clear_meas();
}
