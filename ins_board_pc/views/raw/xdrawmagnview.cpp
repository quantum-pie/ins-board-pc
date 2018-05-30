#include "views/raw/xdrawmagnview.h"

#include <QWidget>
#include <QGridLayout>

XDRawMagnView::XDRawMagnView(QWidget *dummy_plot, QGridLayout *container_layout, QString title)
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

    data.reserve(5000);
    proxy->resetArray(&data);
}

void XDRawMagnView::update(const ViewModel & vm)
{
    Vector3D m = vm.m;
    plot.seriesList().at(0)->dataProxy()->addItem(QVector3D(m[0], m[1], m[2]));
}

void XDRawMagnView::clear()
{
    data.resize(0);
}
