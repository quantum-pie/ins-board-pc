#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDataStream>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    udp_socket = new QUdpSocket(this);
    udp_socket->bind(QHostAddress("192.168.4.1"), 65000);

    connect(udp_socket, SIGNAL(readyRead()), this, SLOT(read_datagrams()));
}

void MainWindow::read_datagrams()
{
    while (udp_socket->hasPendingDatagrams())
    {
        QNetworkDatagram datagram = udp_socket->receiveDatagram();
        process_data(datagram.data());
    }
}

void MainWindow::process_data(const QByteArray & data)
{
    QDataStream ds(data);
    ds.setByteOrder(QDataStream::LittleEndian);
    size_t samples = (data.size() - pkt_header_size) / sample_size;

    int pkt;
    ds >> pkt;

    for(size_t i = 0; i < samples; ++i)
    {
        input_t in;
        ds >> in.et >> in.w_x >> in.w_y >> in.w_z >>
              in.a_x >> in.a_y >> in.a_z >>
              in.m_x >> in.m_y >> in.m_z;
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}
