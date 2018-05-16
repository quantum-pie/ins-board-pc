#ifndef FILTERINGTAB_H
#define FILTERINGTAB_H

#include "qualitycontrol.h"

#include <QObject>
#include <Qt3DCore/QTransform>
#include <Qt3DExtras/Qt3DWindow>

#include "qcustomplot.h"

#include <memory>

// TODO fix comments
class BaseFilteringModel;

class BaseFilteringView : public QObject
{
    Q_OBJECT

public:
    BaseFilteringView(std::shared_ptr<BaseFilteringModel> model,
                        QGridLayout * view_layout,
                        QWidget * xdplot_container,
                        QCustomPlot * rpy_plot,
                        QCustomPlot * enu_plot,
                        QObject * parent = 0);

private:
    std::shared_ptr<BaseFilteringModel> model;

    QualityControl<Vector3D> rpy_ctrl;

    std::unique_ptr<Qt3DExtras::Qt3DWindow> orient_window;  //!< Pointer to rigid body orientation graphic in Kalman tab.
    Qt3DCore::QTransform body_transform;       //!< Pointer to rigid body box transform on Kalman tab.
    Qt3DCore::QTransform sphere_transform;     //!< Pointer to rigid body sphere transform on Kalman tab.

    QCustomPlot * rpy_plot;
    QCustomPlot * enu_plot;

    QCPCurve raw_track;
    QCPCurve smooth_track;
}

#endif // FILTERINGTAB_H
