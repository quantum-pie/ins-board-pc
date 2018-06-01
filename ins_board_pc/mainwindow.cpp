#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDataStream>
#include <QDebug>
#include <QtNetwork>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    udp_socket = new QUdpSocket(this);
    udp_socket->bind(QHostAddress("192.168.0.100"), 7700);
    connect(udp_socket, SIGNAL(readyRead()), this, SLOT(read_datagrams()));

    resize(1300, 800);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::read_datagrams()
{
    while(udp_socket->hasPendingDatagrams())
    {
        QNetworkDatagram datagram = udp_socket->receiveDatagram();
        process_data(datagram.data());
    }
}

void MainWindow::process_data(const QByteArray & data)
{
    cast_filters();

    QDataStream ds(data);
    ds.setByteOrder(QDataStream::LittleEndian);
    size_t samples = (data.size() - pkt_header_size) / sample_size;

    int pkt;
    ds >> pkt;

    input_t in;
    for(size_t i = 0; i < samples; ++i)
    {
        ds >>   in.new_fix >> in.et >> in.w_x >> in.w_y >> in.w_z >>
                in.a_x >> in.a_y >> in.a_z >>
                in.m_x >> in.m_y >> in.m_z >>
                in.gps.fix >>
                in.gps.time.year >> in.gps.time.month >> in.gps.time.day >>
                in.gps.time.hour >> in.gps.time.minute >> in.gps.time.second >> in.gps.time.msecond >>
                in.gps.lat >> in.gps.lon >>
                in.gps.alt >> in.gps.msl_alt >>
                in.gps.x >> in.gps.y >> in.gps.z >>
                in.gps.vx >> in.gps.vy >> in.gps.vz;

        Filter::FilterInput z = parse_input(in);
        if(ui->pushButton_2->isChecked() && in.gps.fix)
        {
            curr_of->step(z);
            if(ui->comboBox->currentIndex() == 0)
            {
                curr_pf->step(z);
            }
        }
        else if(ui->pushButton_4->isChecked())
        {
            compl_of->step(z);
            if(in.gps.fix)
            {
                dynamic_cast<PositionLKF *>(compl_pf)->step(z);
            }
        }
    }

    if(ui->tabWidget->currentIndex() == 0)
    {
        update_raw_tab(in);
    }

    if(ui->tabWidget->currentIndex() == 1 && ui->pushButton->isChecked())
    {
        update_calibration_tab(in);
    }

    if(ui->tabWidget->currentIndex() == 2 && in.gps.fix)
    {
        update_gps_tab(in);
    }

    if(ui->tabWidget->currentIndex() == 3)
    {
        if(!orient_window_kalman)
        {
            orient_window_kalman = new Qt3DExtras::Qt3DWindow;
            init_orient_plot(ui->dwidget, ui->gridLayout_3,
                             orient_window_kalman, body_transform_kalman, sphere_transform_kalman);
        }

        if(ui->pushButton_2->isChecked())
        {
            update_kalman_tab(in);
        }
    }

    if(ui->tabWidget->currentIndex() == 4)
    {
        if(!orient_window_compl)
        {
            orient_window_compl = new Qt3DExtras::Qt3DWindow;
            init_orient_plot(ui->dwidget2, ui->gridLayout_8,
                             orient_window_compl, body_transform_compl, sphere_transform_compl);
        }

        if(ui->pushButton_4->isChecked())
        {
            update_comp_pos_tab(in);
        }
    }
}


void MainWindow::on_tabWidget_currentChanged(int index)
{

}
