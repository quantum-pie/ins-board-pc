#ifndef MAGNCALIBRATIONCONTROLLER_H
#define MAGNCALIBRATIONCONTROLLER_H

#include "views/base/IBaseView.h"
#include "controllers/observablebase.h"
#include "controllers/runningflag.h"
#include "magncalibrator.h"
#include "packets.h"

#include <QObject>

#include <functional>

class Receiver;
class QPushButton;

class MagnCalibrationController : public QObject,
                                  public ObservableBase<ICalibrationView>,
                                  RunningFlag
{
    Q_OBJECT

public:
    MagnCalibrationController(MagnCalibrator & calibrator, const QPushButton * calibrate_btn, const QPushButton * save_btn);

    using RunningFlag::is_running;
    using RunningFlag::set_running;

public slots:
    void handle_input(const RawPacket & z);
    void handle_calibrate(bool en);
    void save_calibration();

private:
    std::reference_wrapper<MagnCalibrator> calibration_model;
};

#endif // MAGNCALIBRATIONCONTROLLER_H
