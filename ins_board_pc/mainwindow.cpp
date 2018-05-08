#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "orientationekf.h"
#include "orientationcomplement.h"
#include "positionlkf.h"
#include "fullekf.h"
#include "fullukf.h"

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
    setup_ui();

    setup_kalman_op();
    setup_kalman_o();
    setup_kalman_p();
    setup_complementary();

    cast_filters();

    udp_socket = new QUdpSocket(this);
    udp_socket->bind(QHostAddress("192.168.0.100"), 7700);
    connect(udp_socket, SIGNAL(readyRead()), this, SLOT(read_datagrams()));

    resize(1300, 800);
}

MainWindow::~MainWindow()
{
    delete udp_socket;

    for(auto * ptr : filters.values())
        delete ptr;

    delete ui;
}

void MainWindow::setup_ui()
{
    ui->setupUi(this);

    init_graphs();

    magnet_plot = new Q3DScatter;
    magnet_data = new QScatterDataArray;

    magnet_plot_cb = new Q3DScatter;
    magnet_data_cb = new QScatterDataArray;

    init_magnet_plot(ui->widget, magnet_data, magnet_plot, "Magnetometer Raw Measurements");
    init_magnet_plot(ui->widget_2, magnet_data_cb, magnet_plot_cb, "Magnetometer Calibrated");

    body_transform_kalman = new Qt3DCore::QTransform;
    sphere_transform_kalman = new Qt3DCore::QTransform;

    body_transform_compl = new Qt3DCore::QTransform;
    sphere_transform_compl = new Qt3DCore::QTransform;

    ui->gyro_process_le->setText(QString::number(proc_gyro_std));
    ui->gyro_bias_process_le->setText(QString::number(proc_gyro_bias_std));
    ui->accel_process_le->setText(QString::number(proc_accel_std));

    ui->accel_meas_le->setText(QString::number(meas_accel_std));
    ui->magn_meas_le->setText(QString::number(meas_magn_std));
    ui->pos_meas_le->setText(QString::number(meas_gps_cep));
    ui->vel_meas_le->setText(QString::number(meas_gps_vel_abs_std));

    ui->qs_init_le->setText(QString::number(cov_qs_std));
    ui->qx_init_le->setText(QString::number(cov_qx_std));
    ui->qy_init_le->setText(QString::number(cov_qy_std));
    ui->qz_init_le->setText(QString::number(cov_qz_std));
    ui->bias_init_le->setText(QString::number(cov_bias_std));
    ui->pos_init_le->setText(QString::number(cov_pos_std));
    ui->vel_init_le->setText(QString::number(cov_vel_std));
    ui->accel_init_le->setText(QString::number(cov_accel_std));

    ui->samples_le->setText(QString::number(roll_ctrl_kalman.get_sampling()));

    ui->a_gain_le->setText(QString::number(static_accel_gain));
    ui->m_gain_le->setText(QString::number(static_magn_gain));

    ui->accel2_proc_le->setText(QString::number(proc_accel_std));
    ui->pos2_meas_le->setText(QString::number(meas_gps_cep));
    ui->vel2_meas_le->setText(QString::number(meas_gps_vel_abs_std));

    ui->pos2_init_le->setText(QString::number(cov_pos_std));
    ui->vel2_init_le->setText(QString::number(cov_vel_std));
    ui->accel2_init_le->setText(QString::number(cov_accel_std));

    ui->samples_le_2->setText(QString::number(roll_ctrl_compl.get_sampling()));
}

void MainWindow::update_raw_tab(const input_t & in)
{
    update_plot(ui->plot1, QVector3D(in.a_x * 1e3, in.a_y * 1e3, in.a_z * 1e3));
    update_plot(ui->plot2, QVector3D(in.w_x, in.w_y, in.w_z));
    update_plot(ui->plot3, QVector3D(in.m_x, in.m_y, in.m_z));
}

void MainWindow::update_calibration_tab(const input_t & in)
{
    magnet_plot->seriesList().at(0)->dataProxy()->addItem(QVector3D(in.m_x, in.m_z, in.m_y));

    NumVector m(3);
    m << in.m_x, in.m_y, in.m_z;
    magn_cal.update(m);
}

void MainWindow::update_gps_tab(const input_t & in)
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

void MainWindow::update_kalman_tab(const input_t & in)
{
    if(curr_of->is_initialized())
    {
        NumVector rpy = curr_of->get_rpy();

        update_plot(ui->plot4, QVector3D(qRadiansToDegrees(rpy[0]), qRadiansToDegrees(rpy[1]), qRadiansToDegrees(rpy[2])));

        NumVector quat = curr_of->get_orientation_quaternion();
        QQuaternion qquat(quat[0], quat[1], quat[2], quat[3]);

        update_body_transform(qquat, body_transform_kalman, sphere_transform_kalman);

        roll_ctrl_kalman.update(qRadiansToDegrees(rpy[0]));
        pitch_ctrl_kalman.update(qRadiansToDegrees(rpy[1]));
        yaw_ctrl_kalman.update(qRadiansToDegrees(rpy[2]));

        if(roll_ctrl_kalman.is_saturated())
        {
            ui->roll_std_le->setText(QString::number(roll_ctrl_kalman.get_std(), 'f', 2));
            ui->pitch_std_le->setText(QString::number(pitch_ctrl_kalman.get_std(), 'f', 2));
            ui->yaw_std_le->setText(QString::number(yaw_ctrl_kalman.get_std(), 'f', 2));
        }

        ui->magnetic_heading_le->setText(QString::number(qRadiansToDegrees(rpy[2]), 'f', 2));
    }

    if(curr_pf->is_initialized())
    {
        NumVector enu = curr_pf->get_position_enu();

        NumVector raw_pos(3);
        raw_pos << in.gps.x, in.gps.y, in.gps.z;
        NumVector enu_raw = curr_pf->get_position_enu(raw_pos);

        kalman_smooth_track->addData(enu[0], enu[1]);
        kalman_raw_track->addData(enu_raw[0], enu_raw[1]);

        update_enu_plot(ui->plot5);

        double track_angle = curr_pf->get_track_angle();
        ui->gps_heading_le->setText(QString::number(qRadiansToDegrees(track_angle), 'f', 2));
    }
}

void MainWindow::update_comp_pos_tab(const input_t & in)
{
    if(compl_of->is_initialized())
    {
        NumVector rpy = compl_of->get_rpy();

        update_plot(ui->plot6, QVector3D(qRadiansToDegrees(rpy[0]), qRadiansToDegrees(rpy[1]), qRadiansToDegrees(rpy[2])));

        NumVector quat = compl_of->get_orientation_quaternion();
        QQuaternion qquat(quat[0], quat[1], quat[2], quat[3]);

        update_body_transform(qquat, body_transform_compl, sphere_transform_compl);

        roll_ctrl_compl.update(qRadiansToDegrees(rpy[0]));
        pitch_ctrl_compl.update(qRadiansToDegrees(rpy[1]));
        yaw_ctrl_compl.update(qRadiansToDegrees(rpy[2]));

        if(roll_ctrl_compl.is_saturated())
        {
            ui->roll_std_le_2->setText(QString::number(roll_ctrl_compl.get_std(), 'f', 2));
            ui->pitch_std_le_2->setText(QString::number(pitch_ctrl_compl.get_std(), 'f', 2));
            ui->yaw_std_le_2->setText(QString::number(yaw_ctrl_compl.get_std(), 'f', 2));
        }

        ui->track_angle_le->setText(QString::number(qRadiansToDegrees(compl_pf->get_track_angle()), 'f', 2));
        ui->ground_speed_le->setText(QString::number(compl_pf->get_ground_speed(), 'f', 2));
    }

    if(compl_pf->is_initialized())
    {
        NumVector enu = compl_pf->get_position_enu();

        NumVector raw_pos(3);
        raw_pos << in.gps.x, in.gps.y, in.gps.z;
        NumVector enu_raw = compl_pf->get_position_enu(raw_pos);

        compl_smooth_track->addData(enu[0], enu[1]);
        compl_raw_track->addData(enu_raw[0], enu_raw[1]);

        update_enu_plot(ui->plot7);
    }
}

void MainWindow::update_body_transform(const QQuaternion & rotator,
                           Qt3DCore::QTransform * body_transform, Qt3DCore::QTransform * sphere_transform)
{
    body_transform->setRotation(rotator);

    QMatrix4x4 m;
    m.rotate(rotator);
    m.translate(QVector3D(0, 5, 0.5));
    sphere_transform->setMatrix(m);
}

void MainWindow::read_datagrams()
{
    while(udp_socket->hasPendingDatagrams())
    {
        QNetworkDatagram datagram = udp_socket->receiveDatagram();
        process_data(datagram.data());
    }
}

void MainWindow::cast_filters()
{
    if(ui->comboBox->currentIndex() == 0)
    {
        curr_of = dynamic_cast<KalmanOrientationFilter *>(filters["kalman_o"]);
        curr_pf = dynamic_cast<KalmanPositionFilter *>(filters["kalman_p"]);
    }
    else
    {
        curr_of = dynamic_cast<KalmanOrientationFilter *>(filters["kalman_op"]);
        curr_pf = dynamic_cast<KalmanPositionFilter *>(filters["kalman_op"]);
    }

    compl_of = dynamic_cast<OrientationCF *>(filters["complement"]);
    compl_pf = dynamic_cast<KalmanPositionFilter *>(filters["kalman_p"]);

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
    ui->plot5->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    ui->plot5->plotLayout()->insertRow(0);
    ui->plot5->plotLayout()->addElement(0, 0, new QCPTextElement(ui->plot5, "ENU relative position"));

    kalman_raw_track = new QCPCurve(ui->plot5->xAxis, ui->plot5->yAxis);
    kalman_raw_track->setName("Raw track");
    kalman_raw_track->setPen(QPen(Qt::blue));

    kalman_smooth_track = new QCPCurve(ui->plot5->xAxis, ui->plot5->yAxis);
    kalman_smooth_track->setName("Smoothed track");
    kalman_smooth_track->setPen(QPen(Qt::red));

    ui->plot5->legend->setVisible(true);

    ui->plot5->xAxis->setRange(-10, 10);
    ui->plot5->xAxis->setLabel("E, m");

    ui->plot5->yAxis->setRange(-10, 10);
    ui->plot5->yAxis->setLabel("N, m");

    ui->plot5->setBackground(Qt::lightGray);
    ui->plot5->axisRect()->setBackground(Qt::black);
    ui->plot5->legend->setBrush(Qt::lightGray);

    //
    ui->plot6->plotLayout()->insertRow(0);
    ui->plot6->plotLayout()->addElement(0, 0, new QCPTextElement(ui->plot6, "Roll Pitch Yaw"));

    ui->plot6->addGraph();
    ui->plot6->addGraph();
    ui->plot6->addGraph();

    ui->plot6->graph(0)->setName("R");
    ui->plot6->graph(1)->setName("P");
    ui->plot6->graph(2)->setName("Y");

    ui->plot6->legend->setVisible(true);

    ui->plot6->graph(0)->setPen(QPen(Qt::blue));
    ui->plot6->graph(1)->setPen(QPen(Qt::red));
    ui->plot6->graph(2)->setPen(QPen(Qt::cyan));

    ui->plot6->xAxis->setRange(0, 200);
    ui->plot6->xAxis->setLabel("packet");

    ui->plot6->yAxis->setRange(-180, 180);
    ui->plot6->yAxis->setLabel("Angle, deg");

    ui->plot6->setBackground(Qt::lightGray);
    ui->plot6->axisRect()->setBackground(Qt::black);
    ui->plot6->legend->setBrush(Qt::lightGray);

    //
    ui->plot7->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    ui->plot7->plotLayout()->insertRow(0);
    ui->plot7->plotLayout()->addElement(0, 0, new QCPTextElement(ui->plot7, "ENU relative position"));

    compl_raw_track = new QCPCurve(ui->plot7->xAxis, ui->plot7->yAxis);
    compl_raw_track->setName("Raw track");
    compl_raw_track->setPen(QPen(Qt::blue));

    compl_smooth_track = new QCPCurve(ui->plot7->xAxis, ui->plot7->yAxis);
    compl_smooth_track->setName("Smoothed track");
    compl_smooth_track->setPen(QPen(Qt::red));

    ui->plot7->legend->setVisible(true);

    ui->plot7->xAxis->setRange(-10, 10);
    ui->plot7->xAxis->setLabel("E, m");

    ui->plot7->yAxis->setRange(-10, 10);
    ui->plot7->yAxis->setLabel("N, m");

    ui->plot7->setBackground(Qt::lightGray);
    ui->plot7->axisRect()->setBackground(Qt::black);
    ui->plot7->legend->setBrush(Qt::lightGray);
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
        plot->graph(0)->data()->clear();
        plot->graph(1)->data()->clear();
        plot->graph(2)->data()->clear();
    }
}

void MainWindow::update_enu_plot(QCustomPlot * plot)
{
    plot->rescaleAxes();

    double expected_y_span = plot->xAxis->range().size() * plot->axisRect()->height() / plot->axisRect()->width();

    if(plot->yAxis->range().size() < expected_y_span)
    {
        plot->yAxis->setScaleRatio(plot->xAxis, 1);
    }
    else
    {
        plot->xAxis->setScaleRatio(plot->yAxis, 1);
    }

    plot->replot();
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

    data->reserve(5000);
    proxy->resetArray(data);
}

void MainWindow::init_orient_plot(QWidget * dummy_container, QGridLayout * layout_container,
                                  Qt3DExtras::Qt3DWindow * plot,
                                  Qt3DCore::QTransform * body_transform, Qt3DCore::QTransform * sphere_transform)
{
    plot = new Qt3DExtras::Qt3DWindow;
    plot->defaultFrameGraph()->setClearColor(QColor(126, 192, 238));
    QWidget *orient_plot_container = QWidget::createWindowContainer(plot);
    Qt3DCore::QEntity *root = new Qt3DCore::QEntity;
    Qt3DExtras::QPhongMaterial *material = new Qt3DExtras::QPhongMaterial(root);
    material->setDiffuse(Qt::red);

    Qt3DCore::QEntity *body = new Qt3DCore::QEntity(root);
    Qt3DExtras::QCuboidMesh *mesh = new Qt3DExtras::QCuboidMesh;

    mesh->setXExtent(5);
    mesh->setYExtent(10);

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

    sphere_transform->setTranslation(QVector3D(0, 5, 0.5));

    sphere->setRadius(0.5);

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

    Qt3DRender::QCamera *camera = plot->camera();
    camera->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    camera->setViewCenter(QVector3D(0, 0, 0));
    camera->setPosition(QVector3D(0, 0, 20.0f));
    camera->rotateAboutViewCenter(QQuaternion::fromAxisAndAngle(1, 0, 0, 79));

    Qt3DExtras::QOrbitCameraController *camController = new Qt3DExtras::QOrbitCameraController(root);
    camController->setLinearSpeed( 50.0f );
    camController->setLookSpeed( 180.0f );
    camController->setCamera(camera);

    plot->setRootEntity(root);

    layout_container->replaceWidget(dummy_container, orient_plot_container);

    plot->show();
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
        /* fit ellips to sphere */
        magn_cal.fit();
        //magn_cal.fit_simple();

        /* draw fitted sphere */
        magnet_data_cb->resize(magnet_data->size());
        for(int i = 0; i < magnet_data->size(); ++i)
        {
            QVector3D item = (*magnet_data)[i].position();
            double tmp = item.y();
            item.setY(item.z());
            item.setZ(tmp);

            item = magn_cal.calibrate(item);
            tmp = item.y();
            item.setY(item.z());
            item.setZ(tmp);

            (*magnet_data_cb)[i].setPosition(item);
        }
        magnet_plot_cb->seriesList().at(0)->dataProxy()->resetArray(magnet_data_cb);

        NumVector b = magn_cal.get_bias();
        NumMatrix s = magn_cal.get_scale();

        ui->xbias_le->setText(QString::number(b[0]));
        ui->ybias_le->setText(QString::number(b[1]));
        ui->zbias_le->setText(QString::number(b[2]));

        ui->xspan_le->setText(QString::number(s(0, 0)));
        ui->yspan_le->setText(QString::number(s(1, 1)));
        ui->zspan_le->setText(QString::number(s(2, 2)));
    }
}

void MainWindow::on_pushButton_2_toggled(bool checked)
{
    if(checked)
    {
        curr_of->reset();
        curr_pf->reset();

        kalman_raw_track->data()->clear();
        kalman_smooth_track->data()->clear();
    }
}

void MainWindow::on_pushButton_4_toggled(bool checked)
{
    if(checked)
    {
        compl_of->reset();
        compl_pf->reset();

        compl_raw_track->data()->clear();
        compl_smooth_track->data()->clear();
    }
}

void MainWindow::on_pushButton_3_clicked() const
{
    magn_cal.save();
}

void MainWindow::on_gyro_process_le_textEdited(const QString &arg1)
{
    curr_of->set_proc_gyro_std(arg1.toDouble());
}

void MainWindow::on_gyro_bias_process_le_textEdited(const QString &arg1)
{
    curr_of->set_proc_gyro_bias_std(arg1.toDouble());
}

void MainWindow::on_accel_process_le_textEdited(const QString &arg1)
{
    curr_pf->set_proc_accel_std(arg1.toDouble());
}

void MainWindow::on_accel_meas_le_textEdited(const QString &arg1)
{
    curr_of->set_meas_accel_std(arg1.toDouble());
}

void MainWindow::on_magn_meas_le_textEdited(const QString &arg1)
{
    curr_of->set_meas_magn_std(arg1.toDouble());
}

void MainWindow::on_pos_meas_le_textEdited(const QString &arg1)
{
    curr_pf->set_meas_pos_std(arg1.toDouble());
}

void MainWindow::on_vel_meas_le_textEdited(const QString &arg1)
{
    curr_pf->set_meas_vel_std(arg1.toDouble());
}

void MainWindow::on_bias_init_le_textEdited(const QString &arg1)
{
    curr_of->set_init_bias_std(arg1.toDouble());
}

void MainWindow::on_pos_init_le_textEdited(const QString &arg1)
{
    curr_pf->set_init_pos_std(arg1.toDouble());
}

void MainWindow::on_vel_init_le_textEdited(const QString &arg1)
{
    curr_pf->set_init_vel_std(arg1.toDouble());
}

void MainWindow::on_accel_init_le_textEdited(const QString &arg1)
{
    curr_pf->set_init_accel_std(arg1.toDouble());
}

void MainWindow::on_samples_le_textEdited(const QString &arg1)
{
    size_t samples = arg1.toInt();
    roll_ctrl_kalman.set_sampling(samples);
    pitch_ctrl_kalman.set_sampling(samples);
    yaw_ctrl_kalman.set_sampling(samples);
}

void MainWindow::on_a_gain_le_textEdited(const QString &arg1)
{
    compl_of->set_static_accel_gain(arg1.toDouble());
}

void MainWindow::on_m_gain_le_textEdited(const QString &arg1)
{
    compl_of->set_static_magn_gain(arg1.toDouble());
}

void MainWindow::on_samples_le_2_textEdited(const QString &arg1)
{
    size_t samples = arg1.toInt();
    roll_ctrl_compl.set_sampling(samples);
    pitch_ctrl_compl.set_sampling(samples);
    yaw_ctrl_compl.set_sampling(samples);
}

void MainWindow::on_qs_init_le_textEdited(const QString &arg1)
{
    curr_of->set_init_qs_std(arg1.toDouble());
}

void MainWindow::on_qx_init_le_textEdited(const QString &arg1)
{
    curr_of->set_init_qx_std(arg1.toDouble());
}

void MainWindow::on_qy_init_le_textEdited(const QString &arg1)
{
    curr_of->set_init_qy_std(arg1.toDouble());
}

void MainWindow::on_qz_init_le_textEdited(const QString &arg1)
{
    curr_of->set_init_qz_std(arg1.toDouble());
}

void MainWindow::on_accel2_proc_le_textEdited(const QString &arg1)
{
    compl_pf->set_proc_accel_std(arg1.toDouble());
}

void MainWindow::on_pos2_meas_le_textEdited(const QString &arg1)
{
    compl_pf->set_meas_pos_std(arg1.toDouble());
}

void MainWindow::on_vel2_meas_le_textEdited(const QString &arg1)
{
    compl_pf->set_meas_vel_std(arg1.toDouble());
}

void MainWindow::on_pos2_init_le_textEdited(const QString &arg1)
{
    compl_pf->set_init_pos_std(arg1.toDouble());
}

void MainWindow::on_vel2_init_le_textEdited(const QString &arg1)
{
    compl_pf->set_init_vel_std(arg1.toDouble());
}

void MainWindow::on_accel2_init_le_textEdited(const QString &arg1)
{
    compl_pf->set_init_accel_std(arg1.toDouble());
}
