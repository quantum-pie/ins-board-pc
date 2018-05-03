#include "magnetic.h"
#include "utils.h"

MAGtype_MagneticModel ** Magnetic::read_models()
{
    MAGtype_MagneticModel ** magnetic_models = new MAGtype_MagneticModel * [1];

    MAG_robustReadMagModels(const_cast<char*>("/home/root/res/WMM.COF"),
                            &magnetic_models, 1);

    return magnetic_models;
}

MAGtype_MagneticModel * Magnetic::initialize_current_model(MAGtype_MagneticModel const * const * magnetic_models)
{
    int n_max = 0;
    if(n_max < magnetic_models[0]->nMax)
        n_max = magnetic_models[0]->nMax;

    int terms = ((n_max + 1) * (n_max + 2) / 2);

    return MAG_AllocateModelMemory(terms);
}

MAGtype_MagneticModel ** Magnetic::magnetic_models = Magnetic::read_models();
MAGtype_MagneticModel *  Magnetic::timed_magnetic_model = Magnetic::initialize_current_model(Magnetic::magnetic_models);

Magnetic::Magnetic(const Ellipsoid & el)
{
	ellip.a = el.a;
	ellip.b = el.b;
	ellip.fla = el.fla;
	ellip.eps = el.eps;
	ellip.epssq = el.epssq;
	ellip.re = el.re;
}

Magnetic::MagneticField Magnetic::measure(const Vector3D & geo, const boost::gregorian::date & day) const
{
    double lat, lon, alt;
    lat = geo[0];
    lon = geo[1];
    alt = geo[2];

    MAGtype_CoordGeodetic geodetic_coord;
    geodetic_coord.UseGeoid = 0;
    geodetic_coord.phi = utils::radians_to_degrees(lat);
    geodetic_coord.lambda = utils::radians_to_degrees(lon);
    geodetic_coord.HeightAboveEllipsoid = alt * 1e-3;

    MAGtype_CoordSpherical spherical_coord;
    MAGtype_GeoMagneticElements result;

    MAG_GeodeticToSpherical(ellip, geodetic_coord, &spherical_coord);

    MAGtype_Date date;
    date.Year = day.year();
    date.Month = day.month();
    date.Day = day.day();

    char err_msg[256];
    MAG_DateToYear(&date, err_msg);
    MAG_TimelyModifyMagneticModel(date, magnetic_models[0], timed_magnetic_model);
    MAG_Geomag(ellip, spherical_coord, geodetic_coord, timed_magnetic_model, &result);

	MagneticField out;
	out.field << result.X, result.Y, result.Z;
	out.field *= 1e-3;
	out.magn = result.F * 1e-3;
	out.horizon_magn = result.H * 1e-3;
	out.decl = utils::degrees_to_radians(result.Decl);
	out.incl = utils::degrees_to_radians(result.Incl);

	return out;
}
