#ifndef RAWMAGNVIEW_H
#define RAWMAGNVIEW_H

#include "views/base/IBaseView.h"
#include "packets.h"

class QCustomPlot;

/*!
 * @brief The raw magnetic field plot view.
 */
struct RawMagnView : IRawView
{
    /*!
     * @brief RawMagnView constructor.
     * @param plot plot pointer.
     */
    RawMagnView(QCustomPlot * plot);

    ~RawMagnView() override = default;

    void update(const ViewModel & pvd) override;
    void clear();

private:
    QCustomPlot * plot;
};

#endif // RAWMAGNVIEW_H
