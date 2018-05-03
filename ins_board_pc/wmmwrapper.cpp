#include "wmmwrapper.h"
#include "physconst.h"

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

void WrapperWMM::measure(const NumVector & geo, QDate day, double & declination, double & inclination, double & magnitude)
{
    double lat, lon, alt;
    lat = geo[0];
    lon = geo[1];
    alt = geo[2];

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
    magnitude = result.F * 1e-3;
}

NumVector WrapperWMM::cartesian_to_geodetic(const NumVector & pos) const
{
    MAGtype_CoordGeodetic geodetic_coord;
    MAG_CartesianToGeodetic(ellip, pos[0] * 1e-3, pos[1] * 1e-3, pos[2] * 1e-3, &geodetic_coord);

    NumVector res(3);
    res[0] = qDegreesToRadians(geodetic_coord.phi);
    res[1] = qDegreesToRadians(geodetic_coord.lambda);
    res[2] = geodetic_coord.HeightAboveEllipsoid * 1e3;

    return res;
}

NumMatrix WrapperWMM::geodetic_to_dcm(const NumVector & geo) const
{
    double lat, lon;
    lat = geo[0];
    lon = geo[1];

    double clat = qCos(lat);
    double slat = qSin(lat);
    double clon = qCos(lon);
    double slon = qSin(lon);

    NumMatrix DCM(3, 3);

    DCM << -slon, clon, 0,
            -clon * slat, -slon * slat, clat,
            clon * clat, slon * clat, slat;

    return DCM;
}

NumVector WrapperWMM::expected_mag(const NumVector & geo, QDate day)
{
    double declination, inclination, magn;
    measure(geo, day, declination, inclination, magn);

    double sdecl = qSin(declination);
    double cdecl = qCos(declination);
    double sincl = qSin(inclination);
    double cincl = qCos(inclination);

    NumVector RES(3);
    RES << sdecl * cincl, cdecl * cincl, -sincl;
    return RES;
}

double WrapperWMM::expected_mag_magnitude(const NumVector & geo, QDate day)
{
    double declination, inclination, magn;
    measure(geo, day, declination, inclination, magn);
    return magn;
}

double WrapperWMM::expected_gravity_accel(const NumVector & geo) const
{
    double lat, alt;
    lat = geo[0];
    alt = geo[2];

    double f = ellip_f();
    double a = ellip_a();
    double a_sq = a * a;
    double slat_sq = qPow(qSin(lat), 2);

    double normal_surface_gravity = phconst::equator_gravity * (1 + phconst::wgs_k * slat_sq) / qSqrt(1 - ellip_epssq() * slat_sq);
    return normal_surface_gravity * (1 - 2 * alt / a * (1 + f + phconst::wgs_m - 2 * f * slat_sq) + 3 * alt * alt / a_sq);
}

NumMatrix WrapperWMM::dcm_lat_partial(const NumVector & geo) const
{
    double lat, lon;
    lat = geo[0];
    lon = geo[1];

    double clat = qCos(lat);
    double slat = qSin(lat);
    double clon = qCos(lon);
    double slon = qSin(lon);

    NumMatrix RES(3, 3);

    RES << 0, 0, 0,
            -clon * clat, -slon * clat, -slat,
            -clon * slat, -slon * slat, clat;

    return RES;
}

NumMatrix WrapperWMM::dcm_lon_partial(const NumVector & geo) const
{
    double lat, lon;
    lat = geo[0];
    lon = geo[1];

    double clat = qCos(lat);
    double slat = qSin(lat);
    double clon = qCos(lon);
    double slon = qSin(lon);

    NumMatrix RES(3, 3);

    RES << -clon, -slon, 0,
            slon * slat, -clon * slat, 0,
            -slon * clat, clon * clat, 0;

    return RES;
}

NumMatrix WrapperWMM::dgeo_dpos(const NumVector & geo) const
{
    double lat, lon, alt;
    lat = geo[0];
    lon = geo[1];
    alt = geo[2];

    double clat = qCos(lat);
    double slat = qSin(lat);
    double clon = qCos(lon);
    double slon = qSin(lon);

    double bracket = qPow(1 - ellip_epssq() * slat * slat, 1.5);
    double norm_rad = ellip_a() / qSqrt(1 - ellip_epssq() * slat * slat);
    double common_mult = bracket / (ellip_a() * (ellip_epssq() - 1) - alt * bracket);
    double common_mult_2 = 1 / (norm_rad + alt);

    NumMatrix RES(3, 3);

    RES(0, 0) = common_mult / (slat * clon);
    RES(0, 1) = common_mult / (slat * slon);
    RES(0, 2) = -common_mult / clat;

    RES(1, 0) = -common_mult_2 / (clat * slon);
    RES(1, 1) = common_mult_2 / (clat * clon);
    RES(1, 2) = 0;

    RES(2, 0) = 1 / (clat * clon);
    RES(2, 1) = 1 / (clat * slon);
    RES(2, 2) = 1 / slat;

    return RES;
}

double WrapperWMM::ellip_a() const
{
    return ellip.a * 1e3;
}

double WrapperWMM::ellip_f() const
{
    return ellip.fla;
}

double WrapperWMM::ellip_epssq() const
{
    return ellip.epssq;
}

double WrapperWMM::earth_rad() const
{
    return ellip.re * 1e3;
}
