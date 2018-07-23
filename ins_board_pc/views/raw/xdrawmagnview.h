#ifndef XDRAWMAGNVIEW_H
#define XDRAWMAGNVIEW_H

#include "views/base/xdaxisscatter.h"
#include "views/base/IBaseView.h"
#include "packets.h"

/*!
 * @brief The raw magnetic field scatterplot.
 */
struct XDRawMagnView : IRawView, private XDAxisScatter
{
    /*!
     * @brief XDRawMagnView constructor.
     * @param dummy_plot preconstructed empty plot.
     * @param container_layout layout containing empty plot.
     */
    XDRawMagnView(QWidget *dummy_plot, QGridLayout *container_layout);

    ~XDRawMagnView() override = default;

    void update(const ViewModel & vm) override;
    void clear() override;
};

#endif // XDRAWMAGNVIEW_H
