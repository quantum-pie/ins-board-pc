#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <boost/numeric/ublas/assignment.hpp>

#include <QDataStream>
#include <QDebug>
#include <QtNetwork>
#include <QtMath>
#include <QDateTime>

#include <QElapsedTimer>

using namespace QtDataVisualization;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    udp_socket = new QUdpSocket(this);
    udp_socket->bind(QHostAddress("192.168.4.1"), 65000);

    init_graphs();

    magnet_plot = new Q3DScatter;
    magnet_data = new QScatterDataArray;

    magnet_plot_cb = new Q3DScatter;
    magnet_data_cb = new QScatterDataArray;

    init_magnet_plot(ui->widget, magnet_data, magnet_plot, "Magnetometer Raw Measurements");
    init_magnet_plot(ui->widget_2, magnet_data_cb, magnet_plot_cb, "Magnetometer Calibrated");

    marg_filt = new QuaternionKalman(0.001, 0.00000000001, 0.001,
                                     0.002, 1.5,
                                     2.5, 0.1);

    connect(udp_socket, SIGNAL(readyRead()), this, SLOT(read_datagrams()));

    resize(1300, 800);
}

MainWindow::~MainWindow()
{
    delete udp_socket;
    delete marg_filt;
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
    QDataStream ds(data);
    ds.setByteOrder(QDataStream::LittleEndian);
    size_t samples = (data.size() - pkt_header_size) / sample_size;

    int pkt;
    ds >> pkt;

    input_t in;
    double et = 0;
    for(size_t i = 0; i < samples; ++i)
    {
        ds >>   in.et >> in.w_x >> in.w_y >> in.w_z >>
                in.a_x >> in.a_y >> in.a_z >>
                in.m_x >> in.m_y >> in.m_z >>
                in.gps.fix >>
                in.gps.time.year >> in.gps.time.month >> in.gps.time.day >>
                in.gps.time.hour >> in.gps.time.minute >> in.gps.time.second >> in.gps.time.msecond >>
                in.gps.lat >> in.gps.lon >>
                in.gps.alt >> in.gps.msl_alt >>
                in.gps.x >> in.gps.y >> in.gps.z >>
                in.gps.vx >> in.gps.vy >> in.gps.vz;

        et += in.et;
    }

    if(ui->tabWidget->currentIndex() == 0)
    {
        update_plot(ui->plot1, QVector3D(in.a_x * 1e3, in.a_y * 1e3, in.a_z * 1e3));
        update_plot(ui->plot2, QVector3D(in.w_x, in.w_y, in.w_z));
        update_plot(ui->plot3, QVector3D(in.m_x, in.m_y, in.m_z));
    }

    if(ui->tabWidget->currentIndex() == 1)
    {
        if(ui->pushButton->isChecked())
        {
            magnet_data->append(QVector3D(in.m_x, in.m_z, in.m_y));
            magnet_plot->seriesList().at(0)->dataProxy()->resetArray(magnet_data);

            magn_cal.update(in.m_x, in.m_y, in.m_z);

            ui->xbias_le->setText(QString::number(magn_cal.get_x_bias()));
            ui->ybias_le->setText(QString::number(magn_cal.get_y_bias()));
            ui->zbias_le->setText(QString::number(magn_cal.get_z_bias()));

            ui->xspan_le->setText(QString::number(magn_cal.get_x_scale()));
            ui->yspan_le->setText(QString::number(magn_cal.get_y_scale()));
            ui->zspan_le->setText(QString::number(magn_cal.get_z_scale()));
        }
    }

    if(ui->tabWidget->currentIndex() == 2)
    {
        if(in.gps.fix)
        {
            ui->x_le->setText(QString::number(in.gps.x, 'f', 2));
            ui->y_le->setText(QString::number(in.gps.y, 'f', 2));
            ui->z_le->setText(QString::number(in.gps.z, 'f', 2));
            ui->vx_le->setText(QString::number(in.gps.vx, 'f', 2));
            ui->vy_le->setText(QString::number(in.gps.vy, 'f', 2));
            ui->vz_le->setText(QString::number(in.gps.vz, 'f', 2));
            ui->lat_le->setText(QString::number(in.gps.lat, 'f', 7));
            ui->lon_le->setText(QString::number(in.gps.lon, 'f', 7));
            ui->alt_le->setText(QString::number(in.gps.alt, 'f', 2));
            ui->msl_alt_le->setText(QString::number(in.gps.msl_alt, 'f', 2));

            QDateTime dt;
            dt.setDate(QDate(in.gps.time.year, in.gps.time.month, in.gps.time.day));
            dt.setTime(QTime(in.gps.time.hour, in.gps.time.minute, in.gps.time.second, in.gps.time.msecond));

            ui->time_le->setText(dt.toString());
        }
    }

    if(ui->tabWidget->currentIndex() == 3)
    {
        if(ui->pushButton_2->isChecked())
        {
            NumVector w(3), a(3), m(3), geo(3), pos(3), v(3);

            w <<= qDegreesToRadians(in.w_x), qDegreesToRadians(in.w_y), qDegreesToRadians(in.w_z);
            a <<= in.a_x, in.a_y, in.a_z;
            m <<= in.m_x, in.m_y, in.m_z;
            geo <<= qDegreesToRadians(in.gps.lat), qDegreesToRadians(in.gps.lon), in.gps.alt;
            pos <<= in.gps.x, in.gps.y, in.gps.z;
            v <<= in.gps.vx, in.gps.vy, in.gps.vz;

            magn_cal.calibrate(m[0], m[1], m[2]);

            QDate day(in.gps.time.year, in.gps.time.month, in.gps.time.day);

            QuaternionKalman::KalmanInput z{w, a, m, day, geo, pos, v, et};

            marg_filt->step(z);

            if(marg_filt->is_initialized())
            {
                double r, p, y;
                marg_filt->get_rpy(r, p, y);
                update_plot(ui->plot4, QVector3D(qRadiansToDegrees(r), qRadiansToDegrees(p), qRadiansToDegrees(y)));
            }
        }
    }
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
    ui->plot1->graph(2)->setPen(QPen(Qt::cyan));

    ui->plot1->xAxis->setRange(0, 200);
    ui->plot1->xAxis->setLabel("packet");

    ui->plot1->yAxis->setRange(-2000, 2000);
    ui->plot1->yAxis->setLabel("Acceleration, mg");

    ui->plot1->setBackground(Qt::lightGray);
    ui->plot1->axisRect()->setBackground(Qt::black);
    ui->plot1->legend->setBrush(Qt::lightGray);

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
    ui->plot2->graph(2)->setPen(QPen(Qt::cyan));

    ui->plot2->xAxis->setRange(0, 200);
    ui->plot2->xAxis->setLabel("packet");

    ui->plot2->yAxis->setRange(-250, 250);
    ui->plot2->yAxis->setLabel("Angular rate, dps");

    ui->plot2->setBackground(Qt::lightGray);
    ui->plot2->axisRect()->setBackground(Qt::black);
    ui->plot2->legend->setBrush(Qt::lightGray);

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
    ui->plot3->graph(2)->setPen(QPen(Qt::cyan));

    ui->plot3->xAxis->setRange(0, 200);
    ui->plot3->xAxis->setLabel("packet");

    ui->plot3->yAxis->setRange(-500, 500);
    ui->plot3->yAxis->setLabel("Magnetic field, uT");

    ui->plot3->setBackground(Qt::lightGray);
    ui->plot3->axisRect()->setBackground(Qt::black);
    ui->plot3->legend->setBrush(Qt::lightGray);

    //
    ui->plot4->plotLayout()->insertRow(0);
    ui->plot4->plotLayout()->addElement(0, 0, new QCPTextElement(ui->plot4, "Roll Pitch Yaw"));

    ui->plot4->addGraph();
    ui->plot4->addGraph();
    ui->plot4->addGraph();

    ui->plot4->graph(0)->setName("R");
    ui->plot4->graph(1)->setName("P");
    ui->plot4->graph(2)->setName("Y");

    ui->plot4->legend->setVisible(true);

    ui->plot4->graph(0)->setPen(QPen(Qt::blue));
    ui->plot4->graph(1)->setPen(QPen(Qt::red));
    ui->plot4->graph(2)->setPen(QPen(Qt::cyan));

    ui->plot4->xAxis->setRange(0, 200);
    ui->plot4->xAxis->setLabel("packet");

    ui->plot4->yAxis->setRange(-180, 180);
    ui->plot4->yAxis->setLabel("Angle, deg");

    ui->plot4->setBackground(Qt::lightGray);
    ui->plot4->axisRect()->setBackground(Qt::black);
    ui->plot4->legend->setBrush(Qt::lightGray);
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

void MainWindow::init_magnet_plot(QWidget * dummy_container, QScatterDataArray * data, Q3DScatter * plot, QString title)
{
    QWidget *magnet_plot_container = QWidget::createWindowContainer(plot);
    magnet_plot_container->setWindowTitle(title);
    ui->gridLayout->replaceWidget(dummy_container, magnet_plot_container);

    QScatterDataProxy *proxy = new QScatterDataProxy;
    QScatter3DSeries *series = new QScatter3DSeries(proxy);
    series->setItemLabelFormat(QStringLiteral("@xTitle: @xLabel @zTitle: @zLabel @yTitle: @yLabel"));
    series->setMeshSmooth(true);
    series->setItemSize(0.05);
    plot->addSeries(series);

    plot->activeTheme()->setType(Q3DTheme::ThemeEbony);
    plot->setShadowQuality(QAbstract3DGraph::ShadowQualitySoftHigh);
    QFont font = plot->activeTheme()->font();
    font.setPointSize(30);
    plot->activeTheme()->setFont(font);
    plot->scene()->activeCamera()->setCameraPreset(Q3DCamera::CameraPresetIsometricLeft);

    plot->axisX()->setTitle("X");
    plot->axisY()->setTitle("Z");
    plot->axisZ()->setTitle("Y");

    plot->setAspectRatio(1);

    data->reserve(1000);
}

void MainWindow::on_pushButton_toggled(bool checked)
{
    if(checked)
    {
        /* start acquisition */
        magn_cal.reset();
        magnet_data->resize(0);
        magnet_data_cb->resize(0);
    }
    else
    {
        magnet_data_cb->resize(magnet_data->size());
        for(int i = 0; i < magnet_data->size(); ++i)
        {
            (*magnet_data_cb)[i].setPosition(magn_cal.calibrate((*magnet_data)[i].position()));
        }
        magnet_plot_cb->seriesList().at(0)->dataProxy()->resetArray(magnet_data_cb);
    }
}

void MainWindow::on_pushButton_2_toggled(bool checked)
{
    if(checked)
    {
        //marg_filt->reset();
    }
}

void MainWindow::on_pushButton_3_clicked()
{
    magn_cal.save();
}
