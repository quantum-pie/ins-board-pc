#ifndef RAWACCELVIEW_H
#define RAWACCELVIEW_H

#include "views/base/IBaseView.h"
#include "packets.h"

class QCustomPlot;

struct RawAccelView : IRawView
{
    RawAccelView(QCustomPlot * plot);
    ~RawAccelView() override = default;

    void update(const ViewModel & pvd) override;
    void clear() override;

private:
    QCustomPlot * plot;
};

#endif // RAWACCELVIEW_H
