#include "views/raw/rawaccelview.h"
#include "views/base/plotsetup.h"
#include "adapters/rawviewmodel.h"
#include "qcustomplot.h"
#include "utils.h"

#include <QLineEdit>

RawAccelView::RawAccelView(QCustomPlot * plot, QLineEdit * x_avg_le, QLineEdit * y_avg_le, QLineEdit * z_avg_le)
    : plot{ plot }, x_avg_le{ x_avg_le }, y_avg_le{ y_avg_le }, z_avg_le{ z_avg_le }
{
    plots::setup_3axis_plot(plot);

    plot->plotLayout()->insertRow(0);
    plot->plotLayout()->addElement(0, 0, new QCPTextElement(plot, "Acceleration sensor data"));

    plot->graph(0)->setName("x");
    plot->graph(1)->setName("y");
    plot->graph(2)->setName("z");

    plot->xAxis->setRange(0, 100);
    plot->xAxis->setLabel("packet");

    plot->yAxis->setRange(-2000, 2080);
    plot->yAxis->setLabel("Acceleration, mg");
}

void RawAccelView::update(const ViewModel & vm)
{
    plots::update_3axis_plot(plot, vm.packet.a * 1e3);
    auto accel_mean = vm.accel_mean;
    x_avg_le->setText(utils::double_view(accel_mean[0], 4));
    y_avg_le->setText(utils::double_view(accel_mean[1], 4));
    z_avg_le->setText(utils::double_view(accel_mean[2], 4));
}

void RawAccelView::clear()
{
    plots::clear_3axis_plot(plot);
}
