#include "views/raw/rawgpsview.h"
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
{}

void RawGPSView::update(const ViewModel & vm)
{
    x_le->setText(utils::double_view(vm.gps_data.pos[0]));
    y_le->setText(utils::double_view(vm.gps_data.pos[1]));
    z_le->setText(utils::double_view(vm.gps_data.pos[2]));
    vx_le->setText(utils::double_view(vm.gps_data.vel[0]));
    vy_le->setText(utils::double_view(vm.gps_data.vel[1]));
    vz_le->setText(utils::double_view(vm.gps_data.vel[2]));
    lat_le->setText(utils::double_view(vm.gps_data.geo[0], 7));
    lon_le->setText(utils::double_view(vm.gps_data.geo[1], 7));
    alt_le->setText(utils::double_view(vm.gps_data.geo[2]));
    msl_alt_le->setText(utils::double_view(vm.gps_data.msl_altitude));

    QDateTime dt;
    dt.setDate(QDate(vm.gps_data.time.year, vm.gps_data.time.month, vm.gps_data.time.day));
    dt.setTime(QTime(vm.gps_data.time.hour, vm.gps_data.time.minute, vm.gps_data.time.second, vm.gps_data.time.ssecond * 10));

    time_le->setText(dt.toString());
}

void RawGPSView::clear()
{

}
