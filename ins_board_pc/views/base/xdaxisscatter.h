#ifndef XDAXISSCATTER_H
#define XDAXISSCATTER_H

#include "eigenaux.h"

#include <QtDataVisualization>

class QWidget;
class QGridLayout;

using namespace QtDataVisualization;

/*!
 * @brief The 3-axis scatterplot base.
 */
struct XDAxisScatter
{
    /*!
     * @brief XDAxisScatter constructor.
     * @param dummy_plot preconstructred empty plot.
     * @param container_layout layout containing empty plot.
     * @param title plot title.
     */
    XDAxisScatter(QWidget *dummy_plot, QGridLayout *container_layout, QString title);

    /*!
     * @brief Update scatterplot.
     * @param p new point.
     */
    void update_scatter(const Vector3D & p);

    /*!
     * @brief Clear scatterplot.
     */
    void clear_scatter();

private:
    QScatterDataArray * data;
    Q3DScatter plot;
};

#endif // CALIBRATIONVIEW_H
