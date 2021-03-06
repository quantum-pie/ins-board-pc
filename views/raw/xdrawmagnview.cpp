#include "views/raw/xdrawmagnview.h"

#include <QWidget>
#include <QGridLayout>

XDRawMagnView::XDRawMagnView(QWidget *dummy_plot, QGridLayout *container_layout)
    : XDAxisScatter{ dummy_plot, container_layout, "Magnetometer measurements"}
{}

void XDRawMagnView::update(const ViewModel & vm)
{
    update_scatter(vm.m);
}

void XDRawMagnView::clear()
{
    clear_scatter();
}
