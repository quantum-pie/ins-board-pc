/*
 * gravity.h
 *
 *      Author: bigaw
 */

#ifndef INCLUDE_GRAVITY_H_
#define INCLUDE_GRAVITY_H_

#include "ellipsoid.h"
#include "eigenaux.h"

/*!
 * @brief Earth gravitational field reference model.
 */
class Gravity
{
public:
    /*!
     * @brief Class constructor.
     * @param ellip reference ellipsoid model.
     */
	explicit Gravity(const Ellipsoid & ellip);

	/*!
	 * @brief Expected gravity magnitude on Earth surface.
	 * @param latitude geodetic latitude.
	 * @return gravity magnitude.
	 */
	double surface_gravity(double latitude) const;

	/*!
	 * @brief Expected gravity magnitude.
	 * @param geo geodetic coordinates.
	 * @return gravity magnitude.
	 */
	double gravity(const Vector3D & geo) const;

    static const double gf;        //!< Standard gravity.

private:
    const Ellipsoid ellip;

    const double k;									//!< Theoretical gravity formula constant.
    const double m;									//!< Yet another theoretical gravity formula constant.

    static const double GM;  //!< Earth gravitational constant (G * Me).
    static const double gp;   //!< Theoretical gravity at pole.
    static const double ge;   //!< Theoretical gravity at equator.
    static const double omega; //!< Earth rotation speed.
};

#endif /* INCLUDE_GRAVITY_H_ */
