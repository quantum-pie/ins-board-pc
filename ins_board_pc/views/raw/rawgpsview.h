#ifndef RAWGPSVIEW_H
#define RAWGPSVIEW_H

#include "views/IBaseView.h"
#include "packets.h"

class QLineEdit;

struct RawGPSView : IRawView
{
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
