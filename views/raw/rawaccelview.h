#ifndef RAWACCELVIEW_H
#define RAWACCELVIEW_H

#include "views/base/IBaseView.h"
#include "packets.h"

class QCustomPlot;
class QLineEdit;

/*!
 * @brief The raw acceleration plot view.
 */
struct RawAccelView : IRawView
{
    /*!
     * @brief RawAccelView constructor.
     * @param plot plot pointer.
     */
    RawAccelView(QCustomPlot * plot,
                 QLineEdit * x_avg_le,
                 QLineEdit * y_avg_le,
                 QLineEdit * z_avg_le);

    ~RawAccelView() override = default;

    void update(const ViewModel & pvd) override;
    void clear() override;

private:
    QCustomPlot * plot;
    QLineEdit * x_avg_le;
    QLineEdit * y_avg_le;
    QLineEdit * z_avg_le;
};

#endif // RAWACCELVIEW_H
