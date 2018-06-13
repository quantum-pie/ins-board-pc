#ifndef ENUPOSITIONVIEW_H
#define ENUPOSITIONVIEW_H

#include "core/IPositionProvider.h"
#include "views/base/IBaseView.h"

#include "qcustomplot.h"

struct ENUPositionView : IPositionView, IRawView
{
    ENUPositionView(QCustomPlot * plot);
    ~ENUPositionView() override = default;

    void update(const IPositionView::ViewModel & pvd) override;
    void update(const IRawView::ViewModel & pvd) override;
    void clear() override;

    void update_track(QCPCurve * track, const Vector3D & point);

private:
    bool is_initialized;
    QCustomPlot * plot;
    QCPCurve * raw_track;
    QCPCurve * smoothed_track;
    Vector3D start_geo;
};

#endif // ENUPOSITIONVIEW_H
