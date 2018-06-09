/*! \file mainwindow.h
  */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ui_mainwindow.h"

#include "receiver.h"
#include "magncalibrator.h"
#include "terminal.h"

#include "filtering/filters/orientationcomplement.h"
#include "filtering/filters/positionsim.h"

#include "controllers/direct/filtering/filteringcontrollersfwd.h"
#include "controllers/direct/rawcontroller.h"
#include "controllers/direct/magncalibrationcontroller.h"
#include "controllers/metacontroller.h"
#include "controllers/direct/attributes/complorientationattrcontroller.h"
#include "controllers/direct/attributes/simpositionattrcontroller.h"
#include "controllers/switches/mixedmodelswitch.h"
#include "controllers/accumviewcontroller.h"

#include <QMainWindow>

#include <memory>

/*!
 * \brief Main window class.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    /*!
     * \brief Main window constructor.
     * \param parent parent widget.
     */
    explicit MainWindow(QWidget *parent = 0);

    ~MainWindow() override = default;

private slots:
    void got_raw_packet(const RawPacket & z);
    void on_tabWidget_currentChanged(int index);

private:
    std::unique_ptr<Ui::MainWindow> ui;                                //!< Pointer to user interface instance.

    MagnCalibrator magn_cal;                            //!< Magnetometer calibrator instance.
    Receiver receiver;
    Terminal terminal;

    OrientationCF ori_cf;
    PositionSim pos_sim;

    //tab1
    std::unique_ptr<RawController> raw_tab_controller;

    //tab2
    std::unique_ptr<RawController> magn_cal_raw_controller;
    std::unique_ptr<MagnCalibrationController> magn_cal_controller;

    //tab3
    std::shared_ptr<OrientationFilteringController> kalman_ori_controller;
    std::shared_ptr<PositionFilteringController> kalman_pos_controller;
    std::unique_ptr<MetaController> meta_controller;
    std::unique_ptr<AccumViewController> kalman_accum_view_controller;

    //tab4
    std::unique_ptr<OrientationFilteringController> compl_ori_controller;
    std::unique_ptr<PositionFilteringController> sim_pos_controller;
    std::unique_ptr<ComplOrientationAttrController> compl_oriattr_controller;
    std::unique_ptr<SimPositionAttrController> sim_posattr_controller;
    std::unique_ptr<AccumViewController> compl_accum_view_controller;

    QLabel * current_time;
};

#endif // MAINWINDOW_H
