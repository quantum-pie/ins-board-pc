#include "views/orientation/rpyremoteview.h"
#include "views/base/plotsetup.h"
#include "qcustomplot.h"
#include "utils.h"

RPYRemoteView::RPYRemoteView(QCustomPlot * plot)
{
    plots::setup_rpy_plot(plot);
}

void RPYRemoteView::update(const ViewModel & vm)
{
    Vector3D rpy;
    rpy << vm.roll, vm.pitch, vm.heading;
    plots::update_3axis_plot(plot, rpy.unaryExpr(&utils::radians_to_degrees));
}

void RPYRemoteView::clear()
{
    plots::clear_3axis_plot(plot);
}
