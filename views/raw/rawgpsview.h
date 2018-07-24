#ifndef RAWGPSVIEW_H
#define RAWGPSVIEW_H

#include "views/base/IBaseView.h"
#include "packets.h"

class QLineEdit;

/*!
 * @brief The raw GPS data table view.
 */
struct RawGPSView : IRawView
{
    /*!
     * @brief RawGPSView constructor.
     * @param x_le X coordinate LineEdit.
     * @param y_le Y coordinate LineEdit.
     * @param z_le Z coordinate LineEdit.
     * @param vx_le velocity X-component LineEdit.
     * @param vy_le velocity Y-component LineEdit.
     * @param vz_le velocity Z-component LineEdit.
     * @param lat_le latitude LineEdit.
     * @param lon_le longitude LineEdit.
     * @param alt_le ellipsoid altitude LineEdit.
     * @param msl_alt_le mean sea level altitude LineEdit.
     * @param time_le current time LineEdit.
     */
    RawGPSView(QLineEdit * x_le, QLineEdit * y_le, QLineEdit * z_le,
               QLineEdit * vx_le, QLineEdit * vy_le, QLineEdit * vz_le,
               QLineEdit * lat_le, QLineEdit * lon_le, QLineEdit * alt_le,
               QLineEdit * msl_alt_le, QLineEdit * time_le);

    ~RawGPSView() override = default;

    void update(const ViewModel & vm) override;
    void clear() override;

private:
    QLineEdit * x_le;
    QLineEdit * y_le;
    QLineEdit * z_le;
    QLineEdit * vx_le;
    QLineEdit * vy_le;
    QLineEdit * vz_le;
    QLineEdit * lat_le;
    QLineEdit * lon_le;
    QLineEdit * alt_le;
    QLineEdit * msl_alt_le;
    QLineEdit * time_le;
};

#endif // RAWGPSVIEW_H
