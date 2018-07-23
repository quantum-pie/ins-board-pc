#ifndef PLOTSETUP_H
#define PLOTSETUP_H

#include "eigenaux.h"

class QCustomPlot;
class QCPCurve;

/*!
 * @brief This namespace holds plotting helper functions.
 */
namespace plots
{

/*!
 * @brief Configure Euler angles plot.
 * @param plot plot pointer.
 */
void setup_rpy_plot(QCustomPlot * plot);

/*!
 * @brief Configure ENU position plot.
 * @param plot plot pointer.
 * @param raw_track raw track pointer.
 * @param smoothed_track filtered track pointer.
 */
void setup_enu_plot(QCustomPlot * plot, QCPCurve * &raw_track, QCPCurve * &smoothed_track);

/*!
 * @brief Configure 3-axis plot.
 * @param plot plot pointer.
 */
void setup_3axis_plot(QCustomPlot * plot);

/*!
 * @brief Update 3-axis plot.
 * @param plot plot pointer.
 * @param vec new vector.
 */
void update_3axis_plot(QCustomPlot * plot, const Vector3D & vec);

/*!
 * @brief Clear 3-axis plot.
 * @param plot plot pointer.
 */
void clear_3axis_plot(QCustomPlot * plot);

/*!
 * @brief Update track.
 * @param plot plot pointer.
 * @param track track pointer.
 * @param point new point.
 */
void update_track(QCustomPlot * plot, QCPCurve * track, const Vector3D & point);

}

#endif // PLOTSETUP_H
