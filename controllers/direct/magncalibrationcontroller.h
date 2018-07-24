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

/*!
 * @brief The MagnCalibrationController class
 * This is the controller of magnetic calibration process.
 */
class MagnCalibrationController : public QObject,
                                  public ObservableBase<ICalibrationView>,
                                  RunningFlag
{
    Q_OBJECT

public:
    /*!
     * @brief MagnCalibrationController constructor.
     * @param calibrator Magnetic calibrator reference.
     * @param calibrate_btn Calibrate widget.
     * @param save_btn Save Calibration widget.
     */
    MagnCalibrationController(MagnCalibrator & calibrator, const QPushButton * calibrate_btn, const QPushButton * save_btn);

    //! Bring is running check to scope.
    using RunningFlag::is_running;

    //! Bring is runnung setter to scope.
    using RunningFlag::set_running;

public slots:
    /*!
     * @brief Handle input packet.
     * @param z raw input packet.
     */
    void handle_input(const RawPacket & z);

    /*!
     * @brief Handle calibrate.
     * @param en calibrate boolean condition.
     */
    void handle_calibrate(bool en);

    /*!
     * @brief Save calibration parameters.
     */
    void save_calibration();

private:
    std::reference_wrapper<MagnCalibrator> calibration_model;
};

#endif // MAGNCALIBRATIONCONTROLLER_H
