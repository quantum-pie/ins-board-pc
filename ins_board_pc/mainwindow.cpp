#include "mainwindow.h"

// tab1
#include "views/raw/rawaccelview.h"
#include "views/raw/rawgyroview.h"
#include "views/raw/rawmagnview.h"

// tab2
#include "views/raw/xdrawmagnview.h"
#include "views/calibration/numcalibrationview.h"
#include "views/calibration/xdcalibrationview.h"

// tab3
#include "views/raw/rawgpsview.h"

// tab4
#include "controllers/direct/attributes/kalmanorientationattrcontroller.h"
#include "controllers/direct/attributes/kalmanpositionattrcontroller.h"
#include "controllers/switches/orientationmodelswitch.h"
#include "controllers/switches/positionmodelswitch.h"


#include "views/orientation/rpyorientationview.h"
#include "views/orientation/xdorientationview.h"
#include "views/orientation/stdorientationview.h"
#include "views/position/enupositionview.h"
#include "views/position/trackpositionview.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(std::make_unique<Ui::MainWindow>()),
    receiver("192.168.0.101", 65000, "192.168.0.100", 7700, magn_cal)
{
    ui->setupUi(this);

    //tab 1
    raw_tab_controller = std::make_unique<RawController>();
    raw_tab_controller->attach_view(std::make_unique<RawAccelView>(ui->plot1));
    raw_tab_controller->attach_view(std::make_unique<RawMagnView>(ui->plot2));
    raw_tab_controller->attach_view(std::make_unique<RawGyroView>(ui->plot3));

    //tab2
    magn_cal_raw_controller = std::make_unique<RawController>(ui->pushButton);
    magn_cal_raw_controller->attach_view(std::make_unique<XDRawMagnView>(ui->widget, ui->gridLayout));

    magn_cal_controller = std::make_unique<MagnCalibrationController>(magn_cal, ui->pushButton, ui->pushButton_3);
    magn_cal_controller->attach_view(std::make_unique<XDCalibrationView>(ui->widget_2, ui->gridLayout));
    magn_cal_controller->attach_view(std::make_unique<NumCalibrationView>(ui->xbias_le, ui->ybias_le, ui->zbias_le,
                                                                          ui->xspan_le, ui->yspan_le, ui->zspan_le));

    //tab3
    gps_raw_controller = std::make_unique<RawController>();
    gps_raw_controller->attach_view(std::make_unique<RawGPSView>(ui->x_le, ui->y_le, ui->z_le, ui->vx_le, ui->vy_le,ui->vz_le,
                                                                 ui->lat_le, ui->lon_le, ui->alt_le, ui->msl_alt_le, ui->time_le));

    //tab4
    kalman_ori_controller = std::make_shared<OrientationFilteringController>(ui->pushButton_2);
    kalman_ori_controller->attach_view(std::make_unique<RPYOrientationView>(ui->plot4));
    kalman_ori_controller->attach_view(std::make_unique<XDOrientationView>(ui->dwidget, ui->gridLayout_3));
    kalman_ori_controller->attach_view(std::make_unique<StdOrientationView>(ui->samples_le, ui->roll_std_le, ui->pitch_std_le, ui->yaw_std_le, ui->magnetic_heading_le));

    kalman_pos_controller = std::make_shared<PositionFilteringController>(ui->pushButton_2);
    kalman_pos_controller->attach_view(std::make_unique<ENUPositionView>(ui->plot5));
    kalman_pos_controller->attach_view(std::make_unique<TrackPositionView>(ui->samples_le, ui->gps_heading_le, ui->ground_speed_le));

    auto kalman_oriattr_controller = std::make_unique<KalmanOrientationAttrController>(ui->gyro_process_le, ui->gyro_bias_process_le,
                                                                                      ui->accel_meas_le, ui->magn_meas_le,
                                                                                      ui->qs_init_le, ui->qx_init_le, ui->qy_init_le,
                                                                                      ui->qz_init_le, ui->bias_init_le);
    auto kalman_posattr_controller = std::make_unique<KalmanPositionAttrController>(ui->accel_process_le, ui->pos_meas_le, ui->vel_meas_le,
                                                                                    ui->pos_init_le, ui->vel_init_le, ui->accel_init_le);

    auto kalman_ori_sw = std::make_shared<OrientationModelSwitch>(ui->comboBox_3, kalman_ori_controller, std::move(kalman_oriattr_controller));
    auto kalman_pos_sw = std::make_shared<PositionModelSwitch>(ui->comboBox_4, kalman_pos_controller, std::move(kalman_posattr_controller));
    auto mix_sw = std::make_unique<MixedModelSwitch>(ui->comboBox_2, kalman_pos_sw, kalman_ori_sw);

    meta_controller = std::make_unique<MetaController>(ui->comboBox, kalman_pos_sw, kalman_ori_sw, std::move(mix_sw), kalman_pos_controller);

    //tab5
    compl_ori_controller = std::make_unique<OrientationFilteringController>(ui->pushButton_4);
    compl_ori_controller->attach_view(std::make_unique<RPYOrientationView>(ui->plot6));
    compl_ori_controller->attach_view(std::make_unique<XDOrientationView>(ui->dwidget2, ui->gridLayout_8));
    compl_ori_controller->attach_view(std::make_unique<StdOrientationView>(ui->samples_le_2, ui->roll_std_le_2, ui->pitch_std_le_2, ui->yaw_std_le_2, ui->magnetic_heading_le_2));

    sim_pos_controller = std::make_unique<PositionFilteringController>(ui->pushButton_4);
    sim_pos_controller->attach_view(std::make_unique<ENUPositionView>(ui->plot7));
    sim_pos_controller->attach_view(std::make_unique<TrackPositionView>(ui->samples_le_2, ui->track_angle_le, ui->ground_speed_le_2));

    compl_oriattr_controller = std::make_unique<ComplOrientationAttrController>(ui->a_gain_le, ui->m_gain_le, ui->b_gain_le);
    sim_posattr_controller = std::make_unique<SimPositionAttrController>(ui->sim_speed_le, ui->init_track_le);

    compl_ori_controller->set_model(&ori_cf);
    compl_oriattr_controller->set_model(&ori_cf);
    sim_pos_controller->set_model(&pos_sim);
    sim_posattr_controller->set_model(&pos_sim);

    resize(1300, 800);
}

void MainWindow::on_tabWidget_currentChanged(int index)
{

}
