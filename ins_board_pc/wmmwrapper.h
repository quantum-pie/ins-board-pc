#ifndef WMMWRAPPER_H
#define WMMWRAPPER_H

extern "C" {
    #include "wmm/GeomagnetismHeader.h"
}

#include "eigenaux.h"

#include <QDate>

class WrapperWMM
{
public:
    static WrapperWMM& instance()
    {
         static WrapperWMM wmm;
         return wmm;
    }

    void measure(double lat, double lon, double alt, QDate day, double & declination, double & inclination, double & magnitude);
    double ellip_a() const;
    double ellip_epssq() const;
    double earth_rad() const;
    double ellip_f() const;

    void cartesian_to_geodetic(const NumVector & pos, double & lat, double & lon, double & alt) const;

    NumMatrix geodetic_to_dcm(double lat, double lon) const;
    NumVector expected_mag(double lat, double lon, double alt, QDate day);
    double expected_mag_magnitude(double lat, double lon, double alt, QDate day);
    double expected_gravity_accel(double lat, double alt) const;

    /* auxiliary derivatives */
    NumMatrix dcm_lat_partial(double lat, double lon) const;
    NumMatrix dcm_lon_partial(double lat, double lon) const;

    /* main derivatives */
    NumMatrix dgeo_dpos(double lat, double lon, double alt) const;

private:
    WrapperWMM();

    MAGtype_Ellipsoid ellip;
    MAGtype_Geoid geoid;
    MAGtype_MagneticModel ** magnetic_models;
    MAGtype_MagneticModel *  timed_magnetic_model;

    char err_msg[255];
};

#endif // WMMWRAPPER_H
