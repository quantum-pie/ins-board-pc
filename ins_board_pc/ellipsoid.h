/*
 * ellipsoid.h
 *
 *      Author: bigaw
 */

#ifndef INCLUDE_ELLIPSOID_H_
#define INCLUDE_ELLIPSOID_H_

/*!
 * @brief Earth reference ellipsoid model.
 */
struct Ellipsoid
{
	/*!
	 * @brief General-purpose class constructor from semi-major axis and flattening.
	 * @param semi_major_axis semi-major axis size.
	 * @param flattening ellipsoid flattening.
	 */
	Ellipsoid(double semi_major_axis, double flattening);

	/*!
	 * @brief Special constructor for spherical Earth models.
	 * @param axis mean Earth radius.
	 */
	explicit Ellipsoid(double axis);

	const double a; 					//!< Semi-major axis of the ellipsoid
	const double fla; 					//!< Flattening
	const double b; 					//!< Semi-minor axis of the ellipsoid
	const double epssq; 				//!< First eccentricity squared
	const double eps; 					//!< First eccentricity
	const double re; 					//!< Mean radius of  ellipsoid

    const double a1;                    //!< Derived a1.
    const double a2;                    //!< Derived a2.
    const double a3;                    //!< Derived a3.
    const double a4;                    //!< Derived a4.
    const double a5;                    //!< Derived a5.
    const double a6;                    //!< Derived a6.

    static const Ellipsoid WGS84;		 //!< Predefined WGS84 ellipsoid.
    static const Ellipsoid sphere;		 //!< Predefined standard spherical Earth model.
};

#endif /* INCLUDE_ELLIPSOID_H_ */
