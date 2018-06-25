/*
 * ellipsoid.cpp
 *
 *      Author: bigaw
 */

#include "ellipsoid.h"

#include <cmath>

const Ellipsoid Ellipsoid::WGS84 {6378137, 1/298.257223560};
const Ellipsoid Ellipsoid::sphere {6371008.8};

Ellipsoid::Ellipsoid(double semi_major_axis, double flattening)
	: a{ semi_major_axis },
	  fla{ flattening },
	  b{ a * (1 - fla) },
      epssq{ 1 - b * b / (a * a) },
	  eps{ std::sqrt(epssq) },
	  re{ (2 * a + b) / 3 },
	  a1{ a * epssq },
	  a2{ a1 * a1 },
	  a3{ a1 * epssq / 2 },
	  a4{ a2 * 2.5 },
	  a5{ a1 + a3 },
	  a6{ 1 - epssq }
{}

Ellipsoid::Ellipsoid(double axis) : Ellipsoid{ axis, 0.0 }
{}
