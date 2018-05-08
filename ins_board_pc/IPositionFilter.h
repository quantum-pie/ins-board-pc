/*
 * IPositionProvider.h
 *
 *      Author: Ermakov_P
 */

#ifndef INCLUDE_IPOSITIONFILTER_H_
#define INCLUDE_IPOSITIONFILTER_H_

#include "eigenaux.h"
#include "IFilter.h"

class Ellipsoid;

/*!
 * @brief Position filter interface.
 */
struct IPositionFilter : virtual IFilter
{
	/*!
	 * @brief Class destructor.
	 */
	~IPositionFilter() override = default;

	/*!
	 * @brief Get ECEF coordinates vector.
	 * @return position vector.
	 */
	virtual Vector3D get_cartesian() const = 0;

    /*!
     * @brief Get underlying Earth ellipsoid model.
     * @return ellipsoid model reference.
     */
    virtual Ellipsoid get_ellipsoid() const = 0;

	/*!
	 * @brief Get ECEF velocity vector.
	 * @return velocity vector.
	 */
	virtual Vector3D get_velocity() const = 0;

	/*!
	 * @brief Get ECEF acceleration vector.
	 * @return acceleration vector.
	 */
	virtual Vector3D get_acceleration() const = 0;
};

#endif /* INCLUDE_IPOSITIONFILTER_H_ */
