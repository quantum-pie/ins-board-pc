#include "views/raw/rawgpsview.h"
#include "adapters/rawviewmodel.h"
#include "utils.h"

#include <QLineEdit>
#include <QDateTime>

RawGPSView::RawGPSView(QLineEdit *x_le, QLineEdit *y_le, QLineEdit *z_le,
                       QLineEdit *vx_le, QLineEdit *vy_le, QLineEdit *vz_le,
                       QLineEdit *lat_le, QLineEdit *lon_le, QLineEdit *alt_le,
                       QLineEdit *msl_alt_le, QLineEdit *time_le)
    : x_le{ x_le }, y_le{ y_le }, z_le{ z_le },
      vx_le{ vx_le }, vy_le{ vy_le }, vz_le{ vz_le},
      lat_le{ lat_le }, lon_le{ lon_le }, alt_le{ alt_le },
      msl_alt_le{ msl_alt_le }, time_le{ time_le }
{
    x_le->setReadOnly(true);
    y_le->setReadOnly(true);
    z_le->setReadOnly(true);
    vx_le->setReadOnly(true);
    vy_le->setReadOnly(true);
    vz_le->setReadOnly(true);
    lat_le->setReadOnly(true);
    lon_le->setReadOnly(true);
    alt_le->setReadOnly(true);
    msl_alt_le->setReadOnly(true);
    time_le->setReadOnly(true);
}

void RawGPSView::update(const ViewModel & vm)
{
    x_le->setText(utils::double_view(vm.packet.gps_data.pos[0]));
    y_le->setText(utils::double_view(vm.packet.gps_data.pos[1]));
    z_le->setText(utils::double_view(vm.packet.gps_data.pos[2]));
    vx_le->setText(utils::double_view(vm.packet.gps_data.vel[0]));
    vy_le->setText(utils::double_view(vm.packet.gps_data.vel[1]));
    vz_le->setText(utils::double_view(vm.packet.gps_data.vel[2]));
    lat_le->setText(utils::double_view(vm.packet.gps_data.geo[0], 7));
    lon_le->setText(utils::double_view(vm.packet.gps_data.geo[1], 7));
    alt_le->setText(utils::double_view(vm.packet.gps_data.geo[2]));
    msl_alt_le->setText(utils::double_view(vm.packet.gps_data.msl_altitude));
    time_le->setText(utils::gps_time_string(vm.packet.gps_data.time));
}

void RawGPSView::clear()
{

}
