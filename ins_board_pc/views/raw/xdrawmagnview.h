#ifndef XDRAWMAGNVIEW_H
#define XDRAWMAGNVIEW_H

#include "views/IBaseView.h"
#include "packets.h"

#include <QtDataVisualization>

class QWidget;
class QGridLayout;

using namespace QtDataVisualization;

struct XDRawMagnView : IRawView
{
    XDRawMagnView(QWidget *dummy_plot, QGridLayout *container_layout, QString title);
    ~XDRawMagnView() override = default;

    void update(const ViewModel & vm) override;
    void clear() override;

private:
    Q3DScatter plot;
    QScatterDataArray data;
};

#endif // XDRAWMAGNVIEW_H
