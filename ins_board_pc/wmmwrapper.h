#ifndef WMMWRAPPER_H
#define WMMWRAPPER_H

extern "C" {
    #include "wmm/GeomagnetismHeader.h"
}

#include <QDate>

class WrapperWMM
{
public:
    WrapperWMM();
    void measure(double lat, double lon, double alt, QDate day, double & declination, double & inclination);
    void cartesian_to_geodetic(double x, double y, double z, double & lat, double & lon, double & alt);
    double ellip_a();
    double ellip_epssq();
    double earth_rad();
    double ellip_f();

private:
    MAGtype_Ellipsoid ellip;
    MAGtype_Geoid geoid;
    MAGtype_MagneticModel ** magnetic_models;
    MAGtype_MagneticModel *  timed_magnetic_model;

    char err_msg[255];
};

#endif // WMMWRAPPER_H
