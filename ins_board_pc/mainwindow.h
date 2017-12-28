#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtNetwork>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void read_datagrams();

private:
    void process_data(const QByteArray & data);

    Ui::MainWindow *ui;
    QUdpSocket *udp_socket;

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
