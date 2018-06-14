#include "views/orientation/rpyorientationview.h"
#include "views/base/plotsetup.h"
#include "adapters/orientationfilteringviewmodel.h"
#include "qcustomplot.h"
#include "utils.h"

RPYOrientationView::RPYOrientationView(QCustomPlot * plot) : plot{ plot }
{
    plots::setup_rpy_plot(plot);
}

void RPYOrientationView::update(const ViewModel & vm)
{
    Vector3D rpy = vm.rpy.unaryExpr(&utils::radians_to_degrees);
    plots::update_3axis_plot(plot, rpy);
}

void RPYOrientationView::clear()
{
    plots::clear_3axis_plot(plot);
}
