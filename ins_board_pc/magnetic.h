/*! @file magnetic.h
  */

#ifndef WMMWRAPPER_H
#define WMMWRAPPER_H

extern "C" {
    #include "wmm/GeomagnetismHeader.h"
}

#include "eigenaux.h"
#include "ellipsoid.h"

#include <boost/date_time/gregorian/gregorian.hpp>

/*!
 * @brief Wrapper class for World Magnetic Model.
 */
class Magnetic
{
public:
	/*!
	 * @brief Magnetic field parameters structure.
	 */
	struct MagneticField
	{
		Vector3D field;			//!< Magnetic field vector.
		double magn;			//!< Magnetic field magnitude.
		double horizon_magn;	//!< Magnitude of magnetic field projection to horizon.
		double decl;			//!< Magnetic field declination angle.
		double incl;			//!< Magnetic field inclination angle.
	};

    /*!
     * @brief Class constructor.
     * @param ellip reference ellipsoid model.
     */
	explicit Magnetic(const Ellipsoid & ellip);

	/*!
	 * @brief Measure expected magnetic field parameters.
	 * @param geo geodetic coordinates.
	 * @param day current date.
	 * @return magnetic field parameters structure.
	 */
	MagneticField measure(const Vector3D & geo, const boost::gregorian::date & day) const;

private:
	/*!
	 * @brief Read magnetic models.
	 * @return array of pointers to magnetic models.
	 */
    static MAGtype_MagneticModel ** read_models();

    /*!
     * @brief Initialize current magnetic model.
     * @return pointer to current magnetic model.
     */
    static MAGtype_MagneticModel *  initialize_current_model(MAGtype_MagneticModel const * const *);

    static MAGtype_MagneticModel ** magnetic_models;       	//!< Array of pointers to the magnetic models.
    static MAGtype_MagneticModel *  timed_magnetic_model;  	//!< Pointer to the current magnetic model.

    MAGtype_Ellipsoid ellip;                        		//!< Ellipsoid parameters instance.
};

#endif // WMMWRAPPER_H
