/*! \file mainwindow.h
  */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "calibrator.h"
#include "abstractkalmanorientationfilter.h"
#include "abstractkalmanpositionfilter.h"
#include "quatcomplement.h"
#include "qualitycontrol.h"

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
     * \brief GPS timestamp type.
     */
    struct gps_time_t
    {
        unsigned short year;    //!< year.
        unsigned char month;    //!< month.
        unsigned char day;      //!< day.
        unsigned char hour;     //!< hours.
        unsigned char minute;   //!< minutes.
        unsigned char second;   //!< seconds.
        unsigned char msecond;  //!< milliseconds.
    };

    /*!
     * \brief GPS input data structure.
     */
    struct gps_input_t
    {
        unsigned char fix;      //!< Quality of position fix.
        gps_time_t time;        //!< GPS timestamp.
        double lat;             //!< Geodetic latitude in deg.
        double lon;             //!< Geodetic longitude in deg.
        double alt;             //!< Geodetic altitude above ellipsoid in m.
        double msl_alt;         //!< Geodetic altitude above mean sea level in m.
        double x;               //!< Cartesian X in m.
        double y;               //!< Cartesian Y in m.
        double z;               //!< Cartesian Z in m.
        double vx;              //!< Cartesian Vx in m/s.
        double vy;              //!< Cartesian Vy in m/s.
        double vz;              //!< Cartesian Vz in m/s.
    };

    /*!
     * \brief Input data structure.
     */
    struct input_t
    {
        double et;              //!< Elapsed time in s.
        double w_x;             //!< X-axis angular rate in dps.
        double w_y;             //!< Y-axis angular rate in dps.
        double w_z;             //!< Z-axis angular rate in dps.
        double a_x;             //!< X-axis accelerometer readings in g.
        double a_y;             //!< Y-axis accelerometer readings in g.
        double a_z;             //!< Z-axis accelerometer readings in g.
        double m_x;             //!< X-axis magnetometer readings in uT.
        double m_y;             //!< Y-axis magnetometer readings in uT.
        double m_z;             //!< Z-axis magnetometer readings in uT.
        gps_input_t gps;        //!< GPS input data.
    };

    /*!
     * \brief Process received datagram.
     * \param data received byte array.
     */
    void process_data(const QByteArray & data);

    /*!
     * \brief Setup orientation + position Kalman filter.
     */
    void setup_kalman_op();

    /*!
     * \brief Setup orientation Kalman filter.
     */
    void setup_kalman_o();

    /*!
     * \brief Setup position Kalman filter.
     */
    void setup_kalman_p();

    /*!
     * \brief Setup complementary orientation filter.
     */
    void setup_complementary();

    /*!
     * \brief Setup custom user interface initial state.
     */
    void setup_ui();

    /*!
     * \brief Dynamicly determine current filter types.
     */
    void cast_filters();

    /*!
     * \brief Convert input data to filter input structure.
     * \param in input data instance.
     * \return filter input instance.
     */
    AbstractFilter::FilterInput parse_input(const input_t & in) const;

    /*!
     * \brief Update tab with raw measurements visualizations.
     * \param in input data instance.
     */
    void update_raw_tab(const input_t & in);

    /*!
     * \brief Update magnetometer calibration tab.
     * \param in input data instance.
     */
    void update_calibration_tab(const input_t & in);

    /*!
     * \brief Update GPS data tab.
     * \param in input data instance.
     */
    void update_gps_tab(const input_t & in);

    /*!
     * \brief Update Kalman filtering tab.
     */
    void update_kalman_tab(const input_t & in);

    /*!
     * \brief Update complementary filtering tab.
     */
    void update_comp_pos_tab(const input_t & in);

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

    Calibrator magn_cal;                                //!< Magnetometer calibrator instance.

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

    QHash<QString, AbstractFilter *> filters;           //!< Hash table with pointers to all filters.

    AbstractKalmanOrientationFilter * curr_of;          //!< Pointer to the filter which is currently used for orientation estimation in Kalman tab.
    AbstractKalmanPositionFilter * curr_pf;             //!< Pointer to the filter which is currently used for position estimation in Kalman tab.
    QuaternionComplement * compl_of;                    //!< Pointer to the filter which is currently used for orientation estimation in Complement tab.
    AbstractKalmanPositionFilter * compl_pf;            //!< Pointer to the filter which is currently used for position estimation in Complement tab.

    QCPCurve * kalman_raw_track;
    QCPCurve * kalman_smooth_track;
    QCPCurve * compl_raw_track;
    QCPCurve * compl_smooth_track;

    const size_t pkt_header_size = 4;                   //!< Size of input packet header.
    const size_t sample_size = 169;                     //!< Size of one input data sample.

    const double proc_gyro_std = 0.0001;                //!< Default gyroscope process noise std.
    const double proc_gyro_bias_std = 0;                //!< Default gyroscope bias process noise std.
    const double proc_accel_std = 0.00001;              //!< Default acceleration process noise std.

    const double meas_accel_std = 0.005;                //!< Default accelerometer measurement noise std.
    const double meas_magn_std = 0.5;                   //!< Default magnetometer measurement noise std.
    const double meas_gps_cep = 2.5;                    //!< Default GPS position CEP.
    const double meas_gps_vel_abs_std = 0.1;            //!< Default GPS velocity measurement std.

    const double cov_qs_std = 0.015;                    //!< Default std of initial qs estimate.
    const double cov_qx_std = 0.0015;                   //!< Default std of initial qx estimate.
    const double cov_qy_std = 0.0015;                   //!< Default std of initial qy estimate.
    const double cov_qz_std = 0.06;                     //!< Default std of initial qz estimate.
    const double cov_bias_std = 0;                      //!< Default std of initial gyro bias estimate.
    const double cov_pos_std = 2.5;                     //!< Default std of initial position estimate.
    const double cov_vel_std = 0.1;                     //!< Default std of initial velocity estimate.
    const double cov_accel_std = 1;                     //!< Default std of initial acceleration estimate.

    const double static_accel_gain = 0.05;              //!< Default static accelerometer gain.
    const double static_magn_gain = 0.0005;             //!< Default static magnetometer gain.
};

#endif // MAINWINDOW_H
