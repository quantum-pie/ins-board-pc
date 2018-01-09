#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "calibrator.h"

#include <QMainWindow>
#include <QVector3D>
#include <QtDataVisualization>

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
    void update_plot(QCustomPlot * plot, QVector3D vec);

    void on_pushButton_toggled(bool checked);

private:
    void process_data(const QByteArray & data);

    Ui::MainWindow *ui;
    QUdpSocket *udp_socket;

    Q3DScatter *magnet_plot;
    QScatterDataArray *magnet_data;

    Q3DScatter *magnet_plot_cb;
    QScatterDataArray *magnet_data_cb;

    Calibrator magn_cal;

    const size_t pkt_header_size = 4;
    const size_t sample_size = 76;

    struct input_t
    {
        int et;
        double w_x;
        double w_y;
        double w_z;
        double a_x;
        double a_y;
        double a_z;
        double m_x;
        double m_y;
        double m_z;
    };
};

#endif // MAINWINDOW_H
