#ifndef WMMWRAPPER_H
#define WMMWRAPPER_H

extern "C" {
    #include "wmm/GeomagnetismHeader.h"
}

#include "ublasaux.h"

#include <QDate>

class WrapperWMM
{
public:
    static WrapperWMM& instance()
    {
         static WrapperWMM wmm;
         return wmm;
    }

    void measure(double lat, double lon, double alt, QDate day, double & declination, double & inclination);
    double ellip_a();
    double ellip_epssq();
    double earth_rad();
    double ellip_f();

    void cartesian_to_geodetic(const NumVector & pos, double & lat, double & lon, double & alt);

    NumMatrix geodetic_to_dcm(double lat, double lon);
    NumVector expected_mag(double lat, double lon, double alt, QDate day);
    double expected_gravity_accel(double lat, double alt);

    /* auxiliary derivatives */
    NumMatrix dcm_lat_partial(double lat, double lon);
    NumMatrix dcm_lon_partial(double lat, double lon);

    /* main derivatives */
    NumMatrix dgeo_dpos(double lat, double lon, double alt);

private:
    WrapperWMM();

    MAGtype_Ellipsoid ellip;
    MAGtype_Geoid geoid;
    MAGtype_MagneticModel ** magnetic_models;
    MAGtype_MagneticModel *  timed_magnetic_model;

    char err_msg[255];
};

#endif // WMMWRAPPER_H
