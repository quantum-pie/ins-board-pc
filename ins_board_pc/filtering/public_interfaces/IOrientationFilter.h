/*
 * IOrientationProvider.h
 *
 *      Author: bigaw
 */

#ifndef INCLUDE_IORIENTATIONFILTER_H_
#define INCLUDE_IORIENTATIONFILTER_H_

#include "filtering/public_interfaces/IFilter.h"

#include "quaternion.h"

/*!
 * @brief Orientation filter interface.
 */
struct IOrientationFilter : virtual IFilter
{
    /*!
     * @brief Get current orientation quaternion.
     * @return vector representing quaternion.
     */
    quat::Quaternion get_orientation_quaternion() const
    {
        return do_get_orientation_quaternion();
    }

    /*!
     * @brief Get current gyroscope bias.
     * @return gyroscope bias vector.
     */
    Vector3D get_gyro_bias() const
    {
        return do_get_gyro_bias();
    }

    /*!
     * @brief Class desctructor.
     */
	~IOrientationFilter() override = default;

private:
    virtual quat::Quaternion do_get_orientation_quaternion() const = 0;
    virtual Vector3D do_get_gyro_bias() const = 0;
};

#endif /* INCLUDE_IORIENTATIONFILTER_H_ */
