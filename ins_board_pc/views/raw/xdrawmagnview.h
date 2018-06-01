#ifndef XDRAWMAGNVIEW_H
#define XDRAWMAGNVIEW_H

#include "views/xdaxisscatter.h"
#include "views/IBaseView.h"
#include "packets.h"

struct XDRawMagnView : IRawView, private XDAxisScatter
{
    XDRawMagnView(QWidget *dummy_plot, QGridLayout *container_layout);
    ~XDRawMagnView() override = default;

    void update(const ViewModel & vm) override;
    void clear() override;
};

#endif // XDRAWMAGNVIEW_H
