#ifndef WMMWRAPPER_H
#define WMMWRAPPER_H

extern "C" {
    #include "wmm/GeomagnetismHeader.h"
}

class WrapperWMM
{
public:
    WrapperWMM();
    void measure(double lat, double lon, double alt, double timestamp, double & declination, double & inclination);
    double ellip_a();
    double ellip_epssq();
    double earth_rad();

private:
    MAGtype_Ellipsoid ellip;
    MAGtype_Geoid geoid;
    MAGtype_MagneticModel ** magnetic_models;
    MAGtype_MagneticModel *  timed_magnetic_model;

    char err_msg[255];
};

#endif // WMMWRAPPER_H
