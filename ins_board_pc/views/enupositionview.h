#ifndef ENUPOSITIONVIEW_H
#define ENUPOSITIONVIEW_H

#include "core/IPositionProvider.h"
#include "views/IBaseView.h"

#include "qcustomplot.h"

struct ENUPositionView : IPositionView
{
    ENUPositionView(QCustomPlot * plot);
    ~ENUPositionView() override = default;
    void update(const ViewModel & pvd) override;
    void clear() override;

private:
    bool is_initialized;
    QCustomPlot * plot;
    QCPCurve raw_track;
    QCPCurve smoothed_track;
    Vector3D start_geo;
};

#endif // ENUPOSITIONVIEW_H
