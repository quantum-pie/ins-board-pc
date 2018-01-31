#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <boost/numeric/ublas/assignment.hpp>

#include <QDataStream>
#include <QDebug>
#include <QtNetwork>
#include <QtMath>
#include <QDateTime>

#include <Qt3DCore/QEntity>
#include <Qt3DRender/QMaterial>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QCameraLens>
#include <Qt3DExtras>

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

    init_orient_plot();

    QuaternionKalman::ProcessNoiseParams proc_params;
    proc_params.gyro_std = 0.05; //!< dps
    proc_params.gyro_bias_std = 1e-8; //!< assume almost constant bias
    proc_params.accel_std = 0.01; //!< m^2/s

    QuaternionKalman::MeasurementNoiseParams meas_params;
    meas_params.accel_std = 0.05; //0.005 //!< g
    meas_params.magn_std = 0.5;//1.2; //!< uT
    meas_params.gps_cep = 2.5;//2.5; //!< m
    meas_params.gps_vel_abs_std = 0.1;//0.1; //!< m/s

    QuaternionKalman::InitCovParams cov_params;
    cov_params.quat_std = 1e-2;
    cov_params.bias_std = 1e-10;
    cov_params.pos_std = 2.5;
    cov_params.vel_std = 0.1;
    cov_params.accel_std = 1;

    ui->gyro_process_le->setText(QString::number(proc_params.gyro_std));
    ui->gyro_bias_process_le->setText(QString::number(proc_params.gyro_bias_std));
    ui->accel_process_le->setText(QString::number(proc_params.accel_std));

    ui->accel_meas_le->setText(QString::number(meas_params.accel_std));
    ui->magn_meas_le->setText(QString::number(meas_params.magn_std));
    ui->pos_meas_le->setText(QString::number(meas_params.gps_cep));
    ui->vel_meas_le->setText(QString::number(meas_params.gps_vel_abs_std));

    ui->quat_init_le->setText(QString::number(cov_params.quat_std));
    ui->bias_init_le->setText(QString::number(cov_params.bias_std));
    ui->pos_init_le->setText(QString::number(cov_params.pos_std));
    ui->vel_init_le->setText(QString::number(cov_params.vel_std));
    ui->accel_init_le->setText(QString::number(cov_params.accel_std));

    marg_filt = new QuaternionKalman(QuaternionKalman::FilterParams{proc_params, meas_params, cov_params});

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

            QuaternionKalman::KalmanInput z{w, a, m, day, geo, pos, v, in.et};

            marg_filt->step(z);
        }
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
            if(marg_filt->is_initialized())
            {
                double r, p, y;
                marg_filt->get_rpy(r, p, y);
                update_plot(ui->plot4, QVector3D(qRadiansToDegrees(r), qRadiansToDegrees(p), qRadiansToDegrees(y)));

                NumVector quat = marg_filt->get_orientation_quaternion();
                body_transform->setRotation(QQuaternion(quat[0], quat[1], quat[2], quat[3]));

                QMatrix4x4 m;
                m.rotate(QQuaternion(quat[0], quat[1], quat[2], quat[3]));
                m.translate(QVector3D(0, 5, 0));
                sphere_transform->setMatrix(m);

                double lat, lon, alt;
                marg_filt->get_geodetic(lat, lon, alt);

                NumVector vel = marg_filt->get_velocity();
                update_plot(ui->plot5, QVector3D(vel[0], vel[1], vel[2]));
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

    //
    //
    ui->plot5->plotLayout()->insertRow(0);
    ui->plot5->plotLayout()->addElement(0, 0, new QCPTextElement(ui->plot5, "Speed"));

    ui->plot5->addGraph();
    ui->plot5->addGraph();
    ui->plot5->addGraph();

    ui->plot5->graph(0)->setName("x");
    ui->plot5->graph(1)->setName("y");
    ui->plot5->graph(2)->setName("z");

    ui->plot5->legend->setVisible(true);

    ui->plot5->graph(0)->setPen(QPen(Qt::blue));
    ui->plot5->graph(1)->setPen(QPen(Qt::red));
    ui->plot5->graph(2)->setPen(QPen(Qt::cyan));

    ui->plot5->xAxis->setRange(0, 200);
    ui->plot5->xAxis->setLabel("packet");

    ui->plot5->yAxis->setRange(-5, 5);
    ui->plot5->yAxis->setLabel("Velocity, m/s");

    ui->plot5->setBackground(Qt::lightGray);
    ui->plot5->axisRect()->setBackground(Qt::black);
    ui->plot5->legend->setBrush(Qt::lightGray);
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

void MainWindow::init_orient_plot()
{
    orient_window = new Qt3DExtras::Qt3DWindow;
    orient_window->defaultFrameGraph()->setClearColor(QColor(126, 192, 238));
    QWidget *orient_plot_container = QWidget::createWindowContainer(orient_window);
    Qt3DCore::QEntity *root = new Qt3DCore::QEntity;
    Qt3DExtras::QPhongMaterial *material = new Qt3DExtras::QPhongMaterial(root);
    material->setDiffuse(Qt::red);

    Qt3DCore::QEntity *body = new Qt3DCore::QEntity(root);
    Qt3DExtras::QCuboidMesh *mesh = new Qt3DExtras::QCuboidMesh;

    mesh->setXExtent(5);
    mesh->setYExtent(10);

    body_transform = new Qt3DCore::QTransform;

    body->addComponent(mesh);
    body->addComponent(body_transform);
    body->addComponent(material);

    Qt3DExtras::QPhongMaterial *plane_material = new Qt3DExtras::QPhongMaterial(root);
    plane_material->setDiffuse(QColor(77, 158, 58));
    plane_material->setSpecular(Qt::white);

    Qt3DCore::QEntity *reference = new Qt3DCore::QEntity(root);
    Qt3DExtras::QPlaneMesh *plane = new Qt3DExtras::QPlaneMesh;

    plane->setHeight(200);
    plane->setWidth(200);

    Qt3DCore::QTransform * plane_transform = new Qt3DCore::QTransform;
    plane_transform->setRotation(QQuaternion::fromAxisAndAngle(1, 0, 0, 90));
    plane_transform->setTranslation(QVector3D(0, 0, -6));

    reference->addComponent(plane);
    reference->addComponent(plane_material);
    reference->addComponent(plane_transform);

    reference->setEnabled(true);

    Qt3DCore::QEntity * north_entity = new  Qt3DCore::QEntity(root);
    Qt3DExtras::QSphereMesh * sphere = new Qt3DExtras::QSphereMesh;

    Qt3DExtras::QPhongMaterial *sphere_material = new Qt3DExtras::QPhongMaterial(root);
    sphere_material->setDiffuse(Qt::blue);

    sphere_transform = new Qt3DCore::QTransform;
    sphere_transform->setTranslation(QVector3D(0, 5, 0));

    sphere->setRadius(1);

    north_entity->addComponent(sphere_transform);
    north_entity->addComponent(sphere);
    north_entity->addComponent(sphere_material);

    north_entity->setEnabled(true);

    //
    Qt3DCore::QEntity * north_line = new  Qt3DCore::QEntity(root);
    Qt3DExtras::QCylinderMesh * nline = new Qt3DExtras::QCylinderMesh;

    Qt3DCore::QTransform * nline_transform = new Qt3DCore::QTransform;
    nline_transform->setTranslation(QVector3D(0, 50, -6));
    //

    nline->setLength(100);
    nline->setRadius(0.2);

    north_line->addComponent(nline_transform);
    north_line->addComponent(nline);
    north_line->addComponent(sphere_material);

    north_line->setEnabled(true);

    //
    Qt3DCore::QEntity * south_line = new  Qt3DCore::QEntity(root);
    Qt3DExtras::QCylinderMesh * sline = new Qt3DExtras::QCylinderMesh;

    Qt3DCore::QTransform * sline_transform = new Qt3DCore::QTransform;
    sline_transform->setTranslation(QVector3D(0, -50, -6));
    //

    sline->setLength(100);
    sline->setRadius(0.2);

    south_line->addComponent(sline_transform);
    south_line->addComponent(sline);
    south_line->addComponent(material);

    south_line->setEnabled(true);

    //
    Qt3DCore::QEntity * light_entity = new Qt3DCore::QEntity(root);
    Qt3DRender::QPointLight * light = new Qt3DRender::QPointLight(light_entity);
    light->setColor(Qt::white);
    light->setIntensity(1.);

    Qt3DCore::QTransform * light_transform = new Qt3DCore::QTransform;
    light_transform->setTranslation(QVector3D(0, -20, 20));

    light_entity->addComponent(light_transform);
    light_entity->addComponent(light);

    light_entity->setEnabled(true);

    Qt3DRender::QCamera *camera = orient_window->camera();
    camera->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    camera->setViewCenter(QVector3D(0, 0, 0));
    camera->setPosition(QVector3D(0, 0, 20.0f));
    camera->rotateAboutViewCenter(QQuaternion::fromAxisAndAngle(1, 0, 0, 79));

    Qt3DExtras::QOrbitCameraController *camController = new Qt3DExtras::QOrbitCameraController(root);
    camController->setLinearSpeed( 50.0f );
    camController->setLookSpeed( 180.0f );
    camController->setCamera(camera);

    orient_window->setRootEntity(root);

    ui->gridLayout_3->replaceWidget(ui->dwidget, orient_plot_container);

    orient_window->show();

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
        marg_filt->reset();
    }
}

void MainWindow::on_pushButton_3_clicked()
{
    magn_cal.save();
}

void MainWindow::on_gyro_process_le_textEdited(const QString &arg1)
{
    marg_filt->set_proc_gyro_std(arg1.toDouble());
}

void MainWindow::on_gyro_bias_process_le_textEdited(const QString &arg1)
{
    marg_filt->set_proc_gyro_bias_std(arg1.toDouble());
}

void MainWindow::on_accel_process_le_textEdited(const QString &arg1)
{
    marg_filt->set_proc_accel_std(arg1.toDouble());
}

void MainWindow::on_accel_meas_le_textEdited(const QString &arg1)
{
    marg_filt->set_meas_accel_std(arg1.toDouble());
}

void MainWindow::on_magn_meas_le_textEdited(const QString &arg1)
{
    marg_filt->set_meas_magn_std(arg1.toDouble());
}

void MainWindow::on_pos_meas_le_textEdited(const QString &arg1)
{
    marg_filt->set_meas_pos_std(arg1.toDouble());
}

void MainWindow::on_vel_meas_le_textEdited(const QString &arg1)
{
    marg_filt->set_meas_vel_std(arg1.toDouble());
}

void MainWindow::on_quat_init_le_textEdited(const QString &arg1)
{
    marg_filt->set_init_quat_std(arg1.toDouble());
}

void MainWindow::on_bias_init_le_textEdited(const QString &arg1)
{
    marg_filt->set_init_bias_std(arg1.toDouble());
}

void MainWindow::on_pos_init_le_textEdited(const QString &arg1)
{
    marg_filt->set_init_pos_std(arg1.toDouble());
}

void MainWindow::on_vel_init_le_textEdited(const QString &arg1)
{
    marg_filt->set_init_vel_std(arg1.toDouble());
}

void MainWindow::on_accel_init_le_textEdited(const QString &arg1)
{
    marg_filt->set_init_accel_std(arg1.toDouble());
}
