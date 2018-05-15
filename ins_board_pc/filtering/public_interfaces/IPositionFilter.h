/*
 * IPositionProvider.h
 *
 *      Author: Ermakov_P
 */

#ifndef INCLUDE_IPOSITIONFILTER_H_
#define INCLUDE_IPOSITIONFILTER_H_

#include "filtering/public_interfaces/IFilter.h"

#include "eigenaux.h"
#include "ellipsoid.h"

/*!
 * @brief Position filter interface.
 */
struct IPositionFilter : virtual IFilter
{
    /*!
     * @brief Get ECEF coordinates vector.
     * @return position vector.
     */
    Vector3D get_cartesian() const
    {
        return do_get_cartesian();
    }

    /*!
     * @brief Get underlying Earth ellipsoid model.
     * @return ellipsoid model reference.
     */
    Ellipsoid get_ellipsoid() const
    {
        return do_get_ellipsoid();
    }

    /*!
     * @brief Get ECEF velocity vector.
     * @return velocity vector.
     */
    Vector3D get_velocity() const
    {
        return do_get_velocity();
    }

    /*!
     * @brief Get ECEF acceleration vector.
     * @return acceleration vector.
     */
    Vector3D get_acceleration() const
    {
        return do_get_acceleration();
    }

	/*!
	 * @brief Class destructor.
	 */
	~IPositionFilter() override = default;

private:
    virtual Vector3D do_get_cartesian() const = 0;
    virtual Ellipsoid do_get_ellipsoid() const = 0;
    virtual Vector3D do_get_velocity() const = 0;
    virtual Vector3D do_get_acceleration() const = 0;
};

#endif /* INCLUDE_IPOSITIONFILTER_H_ */
