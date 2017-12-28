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

    init_graphs();
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

    input_t in;
    for(size_t i = 0; i < samples; ++i)
    {
        ds >> in.et >> in.w_x >> in.w_y >> in.w_z >>
              in.a_x >> in.a_y >> in.a_z >>
              in.m_x >> in.m_y >> in.m_z;
    }

    update_plot(ui->plot1, QVector3D(in.a_x, in.a_y, in.a_z));
    update_plot(ui->plot2, QVector3D(in.w_x * 1e-3, in.w_y  * 1e-3, in.w_z  * 1e-3));
    update_plot(ui->plot3, QVector3D(in.m_x, in.m_y, in.m_z));
}

void MainWindow::init_graphs()
{
    ui->plot1->plotLayout()->insertRow(0);
    ui->plot1->plotLayout()->addElement(0, 0, new QCPTextElement(ui->plot1, "Acceleration sensor data"));

    ui->plot1->addGraph();
    ui->plot1->addGraph();
    ui->plot1->addGraph();

    ui->plot1->graph(0)->setName("x");
    ui->plot1->graph(1)->setName("y");
    ui->plot1->graph(2)->setName("z");

    ui->plot1->legend->setVisible(true);

    ui->plot1->graph(0)->setPen(QPen(Qt::blue));
    ui->plot1->graph(1)->setPen(QPen(Qt::red));
    ui->plot1->graph(2)->setPen(QPen(Qt::black));

    ui->plot1->xAxis->setRange(0, 200);
    ui->plot1->xAxis->setLabel("packet");

    ui->plot1->yAxis->setRange(-2000, 2000);
    ui->plot1->yAxis->setLabel("Acceleration, mg");

    //
    ui->plot2->plotLayout()->insertRow(0);
    ui->plot2->plotLayout()->addElement(0, 0, new QCPTextElement(ui->plot2, "Gyroscope sensor data"));

    ui->plot2->addGraph();
    ui->plot2->addGraph();
    ui->plot2->addGraph();

    ui->plot2->graph(0)->setName("x");
    ui->plot2->graph(1)->setName("y");
    ui->plot2->graph(2)->setName("z");

    ui->plot2->legend->setVisible(true);

    ui->plot2->graph(0)->setPen(QPen(Qt::blue));
    ui->plot2->graph(1)->setPen(QPen(Qt::red));
    ui->plot2->graph(2)->setPen(QPen(Qt::black));

    ui->plot2->xAxis->setRange(0, 200);
    ui->plot2->xAxis->setLabel("packet");

    ui->plot2->yAxis->setRange(-250, 250);
    ui->plot2->yAxis->setLabel("Angular rate, dps");

    //
    ui->plot3->plotLayout()->insertRow(0);
    ui->plot3->plotLayout()->addElement(0, 0, new QCPTextElement(ui->plot3, "Magnetometer sensor data"));

    ui->plot3->addGraph();
    ui->plot3->addGraph();
    ui->plot3->addGraph();

    ui->plot3->graph(0)->setName("x");
    ui->plot3->graph(1)->setName("y");
    ui->plot3->graph(2)->setName("z");

    ui->plot3->legend->setVisible(true);

    ui->plot3->graph(0)->setPen(QPen(Qt::blue));
    ui->plot3->graph(1)->setPen(QPen(Qt::red));
    ui->plot3->graph(2)->setPen(QPen(Qt::black));

    ui->plot3->xAxis->setRange(0, 200);
    ui->plot3->xAxis->setLabel("packet");

    ui->plot3->yAxis->setRange(-500, 500);
    ui->plot3->yAxis->setLabel("Magnetic field, uT");
}

void MainWindow::update_plot(QCustomPlot * plot, QVector3D vec)
{
    int pts = plot->graph(0)->dataCount();
    if(pts < plot->xAxis->range().upper)
    {
        plot->graph(0)->addData(pts, vec.x());
        plot->graph(1)->addData(pts, vec.y());
        plot->graph(2)->addData(pts, vec.z());
        plot->replot();
    }
    else
    {
        plot->graph(0)->setData(QVector<double>(), QVector<double>());
        plot->graph(1)->setData(QVector<double>(), QVector<double>());
        plot->graph(2)->setData(QVector<double>(), QVector<double>());
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}
