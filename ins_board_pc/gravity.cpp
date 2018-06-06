/*
 * gravity.cpp
 *
 *      Author: bigaw
 */

#include "gravity.h"

const double Gravity::gf { 9.80665 };        //!< Standard gravity.
const double Gravity::GM { 3986004.418e8 };  //!< Earth gravitational constant (G * Me).
const double Gravity::gp { 9.8321849378 };   //!< Theoretical gravity at pole.
const double Gravity::ge { 9.7803253359 };   //!< Theoretical gravity at equator.
const double Gravity::omega { 7292115e-11 }; //!< Earth rotation speed.

Gravity::Gravity(const Ellipsoid & e)
	: ellip{ e },
	  k{ (e.b * gp - e.a * ge) / (e.a * ge) },
	  m{ omega * omega * e.a * e.a * e.b / GM }
{}

double Gravity::surface_gravity(double latitude) const
{
    double slat_sq = std::pow(std::sin(latitude), 2);
    return ge * (1 + k * slat_sq) / std::sqrt(1 - ellip.epssq * slat_sq);
}

double Gravity::gravity(const Vector3D & geo) const
{
	double lat, alt;
	lat = geo[0];
	alt = geo[2];

    double slat_sq = std::pow(std::sin(lat), 2);

	double normal_surface_gravity = surface_gravity(lat);
    return normal_surface_gravity * (1 - 2 * alt / ellip.a * (1 + ellip.fla + m - 2 * ellip.fla * slat_sq) + 3 * alt * alt / (ellip.a * ellip.a));
}
