/*! \file physconst.h
  */

#ifndef PHYSCONST_H
#define PHYSCONST_H

//! Module with useful global constants.
namespace phconst
{

const double equator_gravity = 9.7803267714;    //!< theoretical gravity value on the equator.
const double standard_gravity = 9.80665;        //!< mean gravity value on the Earth's surface.
const double wgs_k = 0.00193185265241;          //!< WGS-84 ellipsoid k-parameter.
const double wgs_m = 0.00344978650684;          //!< WGS-84 ellipsoid m-parameter.

}

#endif // PHYSCONST_H
