#ifndef RAWMAGNVIEW_H
#define RAWMAGNVIEW_H

#include "views/base/IBaseView.h"
#include "packets.h"

class QCustomPlot;

struct RawMagnView : IRawView
{
    RawMagnView(QCustomPlot * plot);
    ~RawMagnView() override = default;

    void update(const ViewModel & pvd) override;
    void clear();

private:
    QCustomPlot * plot;
};

#endif // RAWMAGNVIEW_H
