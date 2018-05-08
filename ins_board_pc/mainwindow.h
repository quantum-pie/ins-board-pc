/*! \file mainwindow.h
  */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "IKalmanPositionFilter.h"
#include "IKalmanOrientationFilter.h"
#include "orientationcomplement.h"
#include "fullukf.h"
#include "qualitycontrol.h"
#include "magncalibrator.h"

#include <QMainWindow>
#include <QVector3D>
#include <QtDataVisualization>
#include <QGridLayout>
#include <QHash>

#include <Qt3DCore/QTransform>
#include <Qt3DExtras/Qt3DWindow>

class QCustomPlot;
class QCPCurve;
class QUdpSocket;

class RawPacket;

namespace Ui {
class MainWindow;
}

using namespace QtDataVisualization;

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

    /*!
     * \brief Destructor.
     */
    ~MainWindow();

private slots:
    /*!
     * \brief Read UDP input datagrams.
     */
    void read_datagrams();

    /*!
     * \brief Initialize QCustomPlot graphics.
     */
    void init_graphs();

    /*!
     * \brief Initialize magnetometer calibration 3D graphics.
     * \param dummy_container pointer to holder widget for graphic window.
     * \param data_container pointer to visualized data array.
     * \param plot pointer to 3d graphic.
     * \param title graphic title.
     */
    void init_magnet_plot(QWidget * dummy_container, QScatterDataArray * data_container, Q3DScatter * plot, QString title);

    /*!
     * \brief Initialize rigid body orienation visualizers.
     * \param dummy_container pointer to holder widget for graphic window.
     * \param layout_container pointer to layout containing dummy_widget.
     * \param plot pointer to graphic window.
     * \param body pointer to box transform.
     * \param sphere pointer to sphere transform.
     */
    void init_orient_plot(QWidget * dummy_container, QGridLayout * layout_container,
                          Qt3DExtras::Qt3DWindow * plot, Qt3DCore::QTransform * body, Qt3DCore::QTransform * sphere);

    /*!
     * \brief Update QCustomPlot instance.
     * \param plot pointer to QCustomPlot instance.
     * \param vec vector of data to visualize.
     */
    void update_plot(QCustomPlot * plot, QVector3D vec);

    /*!
     * \brief Update position QCustomPlot instance.
     * \param plot pointer to QCustomPlot instance.
     * \param x x-coordinate.
     * \param y y-coordinate.
     */
    void update_enu_plot(QCustomPlot * plot);

    /*!
     * \brief Calibrate button toggled slot.
     * \param checked button status.
     */
    void on_pushButton_toggled(bool checked);

    /*!
     * \brief Start Kalman filtering button toggled slot.
     * \param checked button status.
     */
    void on_pushButton_2_toggled(bool checked);

    /*!
     * \brief Start complementary filtering button toggled slot.
     * \param checked button status.
     */
    void on_pushButton_4_toggled(bool checked);

    /*!
     * \brief Save calibration parameters button pushed slot.
     */
    void on_pushButton_3_clicked() const;

    /*!
     * \brief Gyroscope process noise std changed slot.
     * \param arg1 new data.
     */
    void on_gyro_process_le_textEdited(const QString &arg1);

    /*!
     * \brief Gyroscope bias process noise std changed slot.
     * \param arg1 new data.
     */
    void on_gyro_bias_process_le_textEdited(const QString &arg1);

    /*!
     * \brief Kalman tab acceleration process noise std changed slot.
     * \param arg1 new data.
     */
    void on_accel_process_le_textEdited(const QString &arg1);

    /*!
     * \brief Accelerometer measurement noise std changed slot.
     * \param arg1 new data.
     */
    void on_accel_meas_le_textEdited(const QString &arg1);

    /*!
     * \brief Magnetometer measurement noise std changed slot.
     * \param arg1 new data.
     */
    void on_magn_meas_le_textEdited(const QString &arg1);

    /*!
     * \brief Kalman tab position measurement noise std changed slot.
     * \param arg1 new data.
     */
    void on_pos_meas_le_textEdited(const QString &arg1);

    /*!
     * \brief Kalman tab velocity measurement noise std changed slot.
     * \param arg1 new data.
     */
    void on_vel_meas_le_textEdited(const QString &arg1);

    /*!
     * \brief Initial gyroscope bias estimate std changed slot.
     * \param arg1 new data.
     */
    void on_bias_init_le_textEdited(const QString &arg1);

    /*!
     * \brief Kalman tab initial position estimate std changed slot.
     * \param arg1 new data.
     */
    void on_pos_init_le_textEdited(const QString &arg1);

    /*!
     * \brief Kalman tab initial velocity estimate std changed slot.
     * \param arg1 new data.
     */
    void on_vel_init_le_textEdited(const QString &arg1);

    /*!
     * \brief Kalman tab initial acceleration estimate std changed slot.
     * \param arg1 new data.
     */
    void on_accel_init_le_textEdited(const QString &arg1);

    /*!
     * \brief Initial qs estimate std changed slot.
     * \param arg1 new data.
     */
    void on_qs_init_le_textEdited(const QString &arg1);

    /*!
     * \brief Initial qx estimate std changed slot.
     * \param arg1 new data.
     */
    void on_qx_init_le_textEdited(const QString &arg1);

    /*!
     * \brief Initial qy estimate std changed slot.
     * \param arg1 new data.
     */
    void on_qy_init_le_textEdited(const QString &arg1);

    /*!
     * \brief Initial qz estimate std changed slot.
     * \param arg1 new data.
     */
    void on_qz_init_le_textEdited(const QString &arg1);

    /*!
     * \brief Kalman tab output angle controller capacity changed slot.
     * \param arg1 new data.
     */
    void on_samples_le_textEdited(const QString &arg1);

    /*!
     * \brief Static accelerometer gain changed slot.
     * \param arg1 new data.
     */
    void on_a_gain_le_textEdited(const QString &arg1);

    /*!
     * \brief Static magnetometer gain changed slot.
     * \param arg1 new data.
     */
    void on_m_gain_le_textEdited(const QString &arg1);

    /*!
     * \brief Complement tab output angle controller capacity changed slot.
     * \param arg1 new data.
     */
    void on_samples_le_2_textEdited(const QString &arg1);

    /*!
     * \brief Complement tab acceleration process noise std changed slot.
     * \param arg1 new data.
     */
    void on_accel2_proc_le_textEdited(const QString &arg1);

    /*!
     * \brief Complement tab position measurement noise std changed slot.
     * \param arg1 new data.
     */
    void on_pos2_meas_le_textEdited(const QString &arg1);

    /*!
     * \brief Complement tab velocity measurement noise std changed slot.
     * \param arg1 new data.
     */
    void on_vel2_meas_le_textEdited(const QString &arg1);

    /*!
     * \brief Complement tab initial position estimate std changed slot.
     * \param arg1 new data.
     */
    void on_pos2_init_le_textEdited(const QString &arg1);

    /*!
     * \brief Complement tab initial velocity estimate std changed slot.
     * \param arg1 new data.
     */
    void on_vel2_init_le_textEdited(const QString &arg1);

    /*!
     * \brief Complement tab initial acceleration estimate std changed slot.
     * \param arg1 new data.
     */
    void on_accel2_init_le_textEdited(const QString &arg1);

private:
    /*!
     * \brief Process received datagram.
     * \param data received byte array.
     */
    void process_data(const QByteArray & data);

    /*!
     * \brief Setup custom user interface initial state.
     */
    void setup_ui();

    /*!
     * \brief Update tab with raw measurements visualizations.
     * \param in input data instance.
     */
    void update_raw_tab(const RawPacket & in);

    /*!
     * \brief Update magnetometer calibration tab.
     * \param in input data instance.
     */
    void update_calibration_tab(const RawPacket & in);

    /*!
     * \brief Update GPS data tab.
     * \param in input data instance.
     */
    void update_gps_tab(const RawPacket & in);

    /*!
     * \brief Update Kalman filtering tab.
     */
    void update_kalman_tab(const RawPacket & in);

    /*!
     * \brief Update complementary filtering tab.
     */
    void update_comp_pos_tab(const RawPacket & in);

    /*!
     * \brief Update rigid body orientation.
     * \param rotator orientation quaternion.
     * \param body_transform pointer to box transform.
     * \param sphere_transform pointer to sphere transform.
     */
    void update_body_transform(const QQuaternion & rotator,
                               Qt3DCore::QTransform * body_transform, Qt3DCore::QTransform * sphere_transform);

    Ui::MainWindow * ui;                                //!< Pointer to user interface instance.
    QUdpSocket * udp_socket;                            //!< Pointer to UDP socket.

    Q3DScatter * magnet_plot;                           //!< Pointer to raw magnetometer data graphic.
    QScatterDataArray * magnet_data;                    //!< Pointer to raw magnetometer data container.

    Q3DScatter * magnet_plot_cb;                        //!< Pointer to calibrated magnetometer data graphic.
    QScatterDataArray * magnet_data_cb;                 //!< Pointer to calibrated magnetometer data container.

    MagnCalibrator magn_cal;                            //!< Magnetometer calibrator instance.

    QualityControl roll_ctrl_kalman;                    //!< Kalman tab output roll controller.
    QualityControl pitch_ctrl_kalman;                   //!< Kalman tab output pitch controller.
    QualityControl yaw_ctrl_kalman;                     //!< Kalman tab output yaw controller.
    QualityControl roll_ctrl_compl;                     //!< Complement tab output roll controller.
    QualityControl pitch_ctrl_compl;                    //!< Complement tab output pitch controller.
    QualityControl yaw_ctrl_compl;                      //!< Complement tab output yaw controller.

    Qt3DExtras::Qt3DWindow * orient_window_kalman = 0;  //!< Pointer to rigid body orientation graphic in Kalman tab.
    Qt3DCore::QTransform * body_transform_kalman;       //!< Pointer to rigid body box transform on Kalman tab.
    Qt3DCore::QTransform * sphere_transform_kalman;     //!< Pointer to rigid body sphere transform on Kalman tab.

    Qt3DExtras::Qt3DWindow * orient_window_compl = 0;   //!< Pointer to rigid body orientation graphic in Complement tab.
    Qt3DCore::QTransform * body_transform_compl;        //!< Pointer to rigid body box transform on Complement tab.
    Qt3DCore::QTransform * sphere_transform_compl;      //!< Pointer to rigid body sphere transform on Complement tab.

    QHash<QString, Filter *> filters;                   //!< Hash table with pointers to all filters.

    KalmanOrientationFilter * curr_of;                  //!< Pointer to the filter which is currently used for orientation estimation in Kalman tab.
    KalmanPositionFilter * curr_pf;                     //!< Pointer to the filter which is currently used for position estimation in Kalman tab.
    OrientationCF * compl_of;                           //!< Pointer to the filter which is currently used for orientation estimation in Complement tab.
    KalmanPositionFilter * compl_pf;                    //!< Pointer to the filter which is currently used for position estimation in Complement tab.

    QCPCurve * kalman_raw_track;
    QCPCurve * kalman_smooth_track;
    QCPCurve * compl_raw_track;
    QCPCurve * compl_smooth_track;



    QualityControl<Vector3D>                                                speed_control;

    static constexpr IKalmanPositionFilter::ProcessNoiseParams              default_pos_proc_noise_params { 0.0001 };
    static constexpr IKalmanPositionFilter::MeasurementNoiseParams          default_pos_meas_noise_params { 0.1, 0.1 };
    static constexpr IKalmanPositionFilter::InitCovParams                   default_pos_init_cov_params { 0.00001, 0.0001, 0.001 };

    static constexpr double default_sim_speed { 30 };
    static constexpr double default_sim_angle { 0 };

    static constexpr FullUKF::UnscentedTransformParams                      default_ut_params{ 0, 2, 1e-3 };

    static constexpr OrientationCF::FilterParams                            default_ori_params { 0.005, 0.00005, 0.00001, 500 };

    static constexpr IKalmanOrientationFilter::ProcessNoiseParams 			default_ori_proc_noise_params { 0.001, 0 };
    static constexpr IKalmanOrientationFilter::MeasurementNoiseParams 		default_ori_meas_noise_params { 0.005, 1.2 };
    static constexpr IKalmanOrientationFilter::InitCovParams 				default_ori_init_cov_params { 0.0001, 0.00001, 0.00001, 0.0001, 0 };
};

#endif // MAINWINDOW_H
