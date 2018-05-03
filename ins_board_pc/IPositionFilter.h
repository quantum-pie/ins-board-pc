/*
 * IPositionProvider.h
 *
 *      Author: Ermakov_P
 */

#ifndef INCLUDE_IPOSITIONFILTER_H_
#define INCLUDE_IPOSITIONFILTER_H_

#include "eigenaux.h"
#include "IFilter.h"

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
	 * @brief Get geodetic coordinates vector.
	 * @return geodetic coordinates.
	 */
	virtual Vector3D get_geodetic() const = 0;

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
