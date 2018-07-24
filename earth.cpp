/*
 * earth.cpp
 *
 *      Author: bigaw
 */

#include "earth.h"

Earth::Earth(const Ellipsoid & e)
	: ellip{ e }, gravity_model{ e }, magnetic_model { e }
{}

Vector3D Earth::magnetic_vector(const Vector3D & geo, const boost::gregorian::date & day) const
{
	Magnetic::MagneticField meas = magnetic_model.measure(geo, day);
	return meas.field;
}

double Earth::magnetic_magnitude(const Vector3D & geo, const boost::gregorian::date & day) const
{
	Magnetic::MagneticField meas = magnetic_model.measure(geo, day);
	return meas.magn;
}

double Earth::magnetic_inclination(const Vector3D & geo, const boost::gregorian::date & day) const
{
	Magnetic::MagneticField meas = magnetic_model.measure(geo, day);
	return meas.incl;
}

double Earth::magnetic_declination(const Vector3D & geo, const boost::gregorian::date & day) const
{
	Magnetic::MagneticField meas = magnetic_model.measure(geo, day);
	return meas.decl;
}

double Earth::gravity(const Vector3D & geo) const
{
	return gravity_model.gravity(geo);
}

Ellipsoid Earth::get_ellipsoid() const
{
	return ellip;
}
