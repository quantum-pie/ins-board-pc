#ifndef RPYORIENTATIONVIEW_H
#define RPYORIENTATIONVIEW_H

#include "views/base/IBaseView.h"
#include "core/IOrientationProvider.h"
#include "packets.h"

class QCustomPlot;

/*!
 * @brief The Euler angles plot orientation view.
 */
struct RPYOrientationView : IOrientationView
{
    /*!
     * @brief RPYOrientationView constructor.
     * @param plot plot pointer.
     */
    RPYOrientationView(QCustomPlot * plot);

    ~RPYOrientationView() override = default;

    void update(const ViewModel & pvd) override;
    void clear() override;

private:
    QCustomPlot * plot;
};

#endif // RPYORIENTATIONVIEW_H
