#ifndef ENUPOSITIONVIEW_H
#define ENUPOSITIONVIEW_H

#include "core/IPositionProvider.h"
#include "views/base/IBaseView.h"

#include "qcustomplot.h"

/*!
 * @brief The ENU plot position view.
 */
struct ENUPositionView : IPositionView, IRawView
{
    /*!
     * @brief ENUPositionView constructor.
     * @param plot plot pointer.
     */
    ENUPositionView(QCustomPlot * plot);

    ~ENUPositionView() override = default;

    void update(const IPositionView::ViewModel & pvd) override;
    void update(const IRawView::ViewModel & pvd) override;
    void clear() override;

private:
    bool is_initialized;
    QCustomPlot * plot;
    QCPCurve * raw_track;
    QCPCurve * smoothed_track;
    Vector3D start_geo;
    Vector3D start_ecef;
};

#endif // ENUPOSITIONVIEW_H
