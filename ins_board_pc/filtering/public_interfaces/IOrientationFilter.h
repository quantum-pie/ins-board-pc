/*
 * IOrientationProvider.h
 *
 *      Author: bigaw
 */

#ifndef INCLUDE_IORIENTATIONFILTER_H_
#define INCLUDE_IORIENTATIONFILTER_H_

#include "filtering/public_interfaces/IFilter.h"
#include "core/IOrientationProviderCore.h"
#include "quaternion.h"

/*!
 * @brief Orientation filter interface.
 */
struct IOrientationFilter : IFilter, private IOrientationProviderCore
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
};

#endif /* INCLUDE_IORIENTATIONFILTER_H_ */
