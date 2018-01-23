#include "wmmwrapper.h"

#include <QtMath>

WrapperWMM::WrapperWMM()
{
    magnetic_models = new MAGtype_MagneticModel*[1];

    MAG_robustReadMagModels(const_cast<char*>("res/WMM.COF"),
                            &magnetic_models, 1);

    int n_max = 0;
    if(n_max < magnetic_models[0]->nMax)
        n_max = magnetic_models[0]->nMax;

    int terms = ((n_max + 1) * (n_max + 2) / 2);

    timed_magnetic_model = MAG_AllocateModelMemory(terms);

    MAG_SetDefaults(&ellip, &geoid);
}

void WrapperWMM::measure(double lat, double lon, double alt, QDate day, double & declination, double & inclination)
{
    MAGtype_CoordGeodetic geodetic_coord;
    geodetic_coord.UseGeoid = 0;
    geodetic_coord.phi = qRadiansToDegrees(lat);
    geodetic_coord.lambda = qRadiansToDegrees(lon);
    geodetic_coord.HeightAboveEllipsoid = alt * 1e-3;

    MAGtype_CoordSpherical spherical_coord;
    MAGtype_GeoMagneticElements result;

    MAG_GeodeticToSpherical(ellip, geodetic_coord, &spherical_coord);

    MAGtype_Date date;
    date.Year = day.year();
    date.Month = day.month();
    date.Day = day.day();

    MAG_DateToYear(&date, err_msg);
    MAG_TimelyModifyMagneticModel(date, magnetic_models[0], timed_magnetic_model);
    MAG_Geomag(ellip, spherical_coord, geodetic_coord, timed_magnetic_model, &result);

    declination = qDegreesToRadians(result.Decl);
    inclination = qDegreesToRadians(result.Incl);
}

void WrapperWMM::cartesian_to_geodetic(double x, double y, double z, double & lat, double & lon, double & alt)
{
    MAGtype_CoordGeodetic geodetic_coord;
    MAG_CartesianToGeodetic(ellip, x * 1e-3, y * 1e-3, z * 1e-3, &geodetic_coord);
    lat = qDegreesToRadians(geodetic_coord.phi);
    lon = qDegreesToRadians(geodetic_coord.lambda);
    alt = geodetic_coord.HeightAboveEllipsoid * 1e3;
}

double WrapperWMM::ellip_a()
{
    return ellip.a * 1e3;
}

double WrapperWMM::ellip_epssq()
{
    return ellip.epssq;
}

double WrapperWMM::earth_rad()
{
    return ellip.re * 1e3;
}
