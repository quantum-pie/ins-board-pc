#include "views/xdaxisscatter.h"

#include <QWidget>
#include <QGridLayout>

XDAxisScatter::XDAxisScatter(QWidget *dummy_plot, QGridLayout *container_layout, QString title)
{
    QWidget *magnet_plot_container = QWidget::createWindowContainer(&plot);
    magnet_plot_container->setWindowTitle(title);
    container_layout->replaceWidget(dummy_plot, magnet_plot_container);

    QScatterDataProxy *proxy = new QScatterDataProxy;
    QScatter3DSeries *series = new QScatter3DSeries(proxy);
    series->setItemLabelFormat(QStringLiteral("@xTitle: @xLabel @zTitle: @zLabel @yTitle: @yLabel"));
    series->setMeshSmooth(true);
    series->setItemSize(0.05);

    plot.addSeries(series);

    plot.activeTheme()->setType(Q3DTheme::ThemeEbony);
    plot.setShadowQuality(QAbstract3DGraph::ShadowQualitySoftHigh);

    QFont font = plot.activeTheme()->font();
    font.setPointSize(30);
    plot.activeTheme()->setFont(font);

    plot.scene()->activeCamera()->setCameraPreset(Q3DCamera::CameraPresetIsometricLeft);

    plot.axisX()->setTitle("X");
    plot.axisY()->setTitle("Z");
    plot.axisZ()->setTitle("Y");

    plot.setAspectRatio(1);

    data = new QScatterDataArray;
    data->reserve(5000);
    proxy->resetArray(data);
}

void XDAxisScatter::update_scatter(const Vector3D & p)
{
    plot.seriesList().at(0)->dataProxy()->addItem(QVector3D(p[0], p[1], p[2]));
}

void XDAxisScatter::clear_scatter()
{
    data->resize(0);
}
