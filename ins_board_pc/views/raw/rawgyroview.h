#ifndef RAWGYROVIEW_H
#define RAWGYROVIEW_H

#include "views/base/IBaseView.h"
#include "packets.h"

class QCustomPlot;

/*!
 * @brief The raw angular rate plot view.
 */
struct RawGyroView : IRawView
{
    /*!
     * @brief RawGyroView constructor.
     * @param plot plot pointer.
     */
    RawGyroView(QCustomPlot * plot);

    ~RawGyroView() override = default;

    void update(const ViewModel & pvd) override;
    void clear() override;

private:
    QCustomPlot * plot;
};

#endif // RAWGYROVIEW_H
