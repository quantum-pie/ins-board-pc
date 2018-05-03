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

    static constexpr double gf { 9.80665 };        //!< Standard gravity.

private:
    const Ellipsoid ellip;

    const double k;									//!< Theoretical gravity formula constant.
    const double m;									//!< Yet another theoretical gravity formula constant.

    static constexpr double GM { 3986004.418e8 };  //!< Earth gravitational constant (G * Me).
    static constexpr double gp { 9.8321849378 };   //!< Theoretical gravity at pole.
    static constexpr double ge { 9.7803253359 };   //!< Theoretical gravity at equator.
    static constexpr double omega { 7292115e-11 }; //!< Earth rotation speed.
};

#endif /* INCLUDE_GRAVITY_H_ */
