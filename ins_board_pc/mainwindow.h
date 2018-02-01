#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "calibrator.h"
#include "quatkalman.h"
#include "qualitycontrol.h"

#include <QMainWindow>
#include <QVector3D>
#include <QtDataVisualization>

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
    void init_orient_plot();
    void update_plot(QCustomPlot * plot, QVector3D vec);

    void on_pushButton_toggled(bool checked);
    void on_pushButton_2_toggled(bool checked);
    void on_pushButton_3_clicked();
    void on_gyro_process_le_textEdited(const QString &arg1);
    void on_gyro_bias_process_le_textEdited(const QString &arg1);
    void on_accel_process_le_textEdited(const QString &arg1);
    void on_accel_meas_le_textEdited(const QString &arg1);
    void on_magn_meas_le_textEdited(const QString &arg1);
    void on_pos_meas_le_textEdited(const QString &arg1);
    void on_vel_meas_le_textEdited(const QString &arg1);
    void on_quat_init_le_textEdited(const QString &arg1);
    void on_bias_init_le_textEdited(const QString &arg1);
    void on_pos_init_le_textEdited(const QString &arg1);
    void on_vel_init_le_textEdited(const QString &arg1);
    void on_accel_init_le_textEdited(const QString &arg1);

    void on_samples_le_textEdited(const QString &arg1);

private:
    void process_data(const QByteArray & data);

    Ui::MainWindow *ui;
    QUdpSocket *udp_socket;

    Q3DScatter *magnet_plot;
    QScatterDataArray *magnet_data;

    Q3DScatter *magnet_plot_cb;
    QScatterDataArray *magnet_data_cb;

    Calibrator magn_cal;

    QuaternionKalman *marg_filt;

    QualityControl roll_ctrl, pitch_ctrl, yaw_ctrl;

    Qt3DExtras::Qt3DWindow *orient_window;
    Qt3DCore::QTransform * body_transform, * sphere_transform;

    const size_t pkt_header_size = 4;
    const size_t sample_size = 169;

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
};

#endif // MAINWINDOW_H
