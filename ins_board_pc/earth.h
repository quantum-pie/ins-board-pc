/*
 * earth.h
 *
 *      Author: bigaw
 */

#ifndef INCLUDE_EARTH_H_
#define INCLUDE_EARTH_H_

#include "ellipsoid.h"
#include "gravity.h"
#include "magnetic.h"
#include "eigenaux.h"

#include <boost/date_time/gregorian/gregorian.hpp>

/*!
 * @brief Earth physical model class.
 *
 * This class provides means of measuring Earth gravitational and magnetic fields as well as
 * inspecting Earth surface ellipsoid parameters.
 */
class Earth
{
public:
    /*!
     * @brief Class constructor.
     * @param ellip reference to Earth surface ellipsoid model.
     */
	explicit Earth(const Ellipsoid & ellip = Ellipsoid::WGS84);

	/*!
	 * @brief Get expected magnetic field vector.
	 * @param geo geodetic coordinates.
	 * @param day date.
	 * @return magnetic field vector.
	 */
    Vector3D magnetic_vector(const Vector3D & geo, const boost::gregorian::date & day) const;

	/*!
	 * @brief Get expected magnetic field vector magnitude.
	 * @param geo geodetic coordinates.
	 * @param day date.
	 * @return magnetic field vector magnitude.
	 */
    double magnetic_magnitude(const Vector3D & geo, const boost::gregorian::date & day) const;

	/*!
	 * @brief Get expected magnetic field inclination.
	 * @param geo geodetic coordinates.
	 * @param day date.
	 * @return magnetic field inclination in radians.
	 */
    double magnetic_inclination(const Vector3D & geo, const boost::gregorian::date & day) const;

	/*!
	 * @brief Get expected magnetic field declination.
	 * @param geo geodetic coordinates.
	 * @param day date.
	 * @return magnetic field declination in radians.
	 */
    double magnetic_declination(const Vector3D & geo, const boost::gregorian::date & day) const;

    /*!
     * @brief Get expected gravity magnitude.
     * @param geo geodetic coordinates.
     * @return expected gravity magnitude.
     */
    double gravity(const Vector3D & geo) const;

    /*!
     * @brief Get reference ellipsoid.
     * @return  reference ellipsoid.
     */
    const Ellipsoid & get_ellipsoid() const;

private:
    const Ellipsoid ellip;			//!< Reference ellipsoid model.
    const Gravity gravity_model;	//!< Reference gravity field model.
    const Magnetic magnetic_model;	//!< Reference magnetic field model.
};

#endif /* INCLUDE_EARTH_H_ */
