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
class QUdpSocket;

namespace Ui {
class MainWindow;
}

using namespace QtDataVisualization;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void read_datagrams();
    void init_graphs();
    void init_magnet_plot(QWidget * dummy_container, QScatterDataArray * data_container, Q3DScatter * plot, QString title);
    void init_orient_plot(QWidget * dummy_container, QGridLayout * layout_container,
                          Qt3DExtras::Qt3DWindow * plot, Qt3DCore::QTransform * body, Qt3DCore::QTransform * sphere);
    void update_plot(QCustomPlot * plot, QVector3D vec);

    void on_pushButton_toggled(bool checked);
    void on_pushButton_2_toggled(bool checked);
    void on_pushButton_3_clicked() const;
    void on_gyro_process_le_textEdited(const QString &arg1);
    void on_gyro_bias_process_le_textEdited(const QString &arg1);
    void on_accel_process_le_textEdited(const QString &arg1);
    void on_accel_meas_le_textEdited(const QString &arg1);
    void on_magn_meas_le_textEdited(const QString &arg1);
    void on_pos_meas_le_textEdited(const QString &arg1);
    void on_vel_meas_le_textEdited(const QString &arg1);
    void on_bias_init_le_textEdited(const QString &arg1);
    void on_pos_init_le_textEdited(const QString &arg1);
    void on_vel_init_le_textEdited(const QString &arg1);
    void on_accel_init_le_textEdited(const QString &arg1);    
    void on_qs_init_le_textEdited(const QString &arg1);
    void on_qx_init_le_textEdited(const QString &arg1);
    void on_qy_init_le_textEdited(const QString &arg1);
    void on_qz_init_le_textEdited(const QString &arg1);

    void on_samples_le_textEdited(const QString &arg1);
    void on_a_gain_le_textEdited(const QString &arg1);
    void on_m_gain_le_textEdited(const QString &arg1);
    void on_samples_le_2_textEdited(const QString &arg1);
    void on_pushButton_4_toggled(bool checked);


private:
    struct gps_time_t
    {
        unsigned short year;
        unsigned char month;
        unsigned char day;
        unsigned char hour;
        unsigned char minute;
        unsigned char second;
        unsigned char msecond;
    };

    struct gps_input_t
    {
        unsigned char fix;
        gps_time_t time;
        double lat;
        double lon;
        double alt;
        double msl_alt;
        double x;
        double y;
        double z;
        double vx;
        double vy;
        double vz;
    };

    struct input_t
    {
        double et;
        double w_x;
        double w_y;
        double w_z;
        double a_x;
        double a_y;
        double a_z;
        double m_x;
        double m_y;
        double m_z;
        gps_input_t gps;
    };

    void process_data(const QByteArray & data);

    void setup_kalman_op();
    void setup_kalman_o();
    void setup_kalman_p();
    void setup_complementary();

    void setup_ui();

    void cast_filters();

    AbstractFilter::FilterInput parse_input(const input_t & in) const;

    void update_raw_tab(const input_t & in);
    void update_calibration_tab(const input_t & in);
    void update_gps_tab(const input_t & in);
    void update_kalman_tab();
    void update_comp_pos_tab();

    void update_body_transform(const QQuaternion & rotator,
                               Qt3DCore::QTransform * body_transform, Qt3DCore::QTransform * sphere_transform);

    Ui::MainWindow * ui;
    QUdpSocket * udp_socket;

    Q3DScatter * magnet_plot;
    QScatterDataArray * magnet_data;

    Q3DScatter * magnet_plot_cb;
    QScatterDataArray * magnet_data_cb;

    Calibrator magn_cal;

    QualityControl roll_ctrl_kalman, pitch_ctrl_kalman, yaw_ctrl_kalman;
    QualityControl roll_ctrl_compl, pitch_ctrl_compl, yaw_ctrl_compl;

    Qt3DExtras::Qt3DWindow * orient_window_kalman = 0;
    Qt3DCore::QTransform * body_transform_kalman, * sphere_transform_kalman;

    Qt3DExtras::Qt3DWindow * orient_window_compl = 0;
    Qt3DCore::QTransform * body_transform_compl, * sphere_transform_compl;

    QHash<QString, AbstractFilter *> filters;

    AbstractKalmanOrientationFilter * curr_of;
    AbstractKalmanPositionFilter * curr_pf;
    QuaternionComplement * compl_of;
    AbstractKalmanPositionFilter * compl_pf;

    const size_t pkt_header_size = 4;
    const size_t sample_size = 169;

    const double proc_gyro_std = 0.0001;
    const double proc_gyro_bias_std = 0;
    const double proc_accel_std = 0.00001;

    const double meas_accel_std = 0.005;
    const double meas_magn_std = 0.5;
    const double meas_gps_cep = 2.5;
    const double meas_gps_vel_abs_std = 0.1;

    const double cov_qs_std = 0.015;
    const double cov_qx_std = 0.0015;
    const double cov_qy_std = 0.0015;
    const double cov_qz_std = 0.06;
    const double cov_bias_std = 0;
    const double cov_pos_std = 2.5;
    const double cov_vel_std = 0.1;
    const double cov_accel_std = 1;

    const double static_accel_gain = 0.05;
    const double static_magn_gain = 0.0005;
};

#endif // MAINWINDOW_H
