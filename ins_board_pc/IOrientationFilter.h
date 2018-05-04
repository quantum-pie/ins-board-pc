/*
 * IOrientationProvider.h
 *
 *      Author: bigaw
 */

#ifndef INCLUDE_IORIENTATIONFILTER_H_
#define INCLUDE_IORIENTATIONFILTER_H_

#include "eigenaux.h"
#include "quaternion.h"
#include "IFilter.h"

/*!
 * @brief Orientation filter interface.
 */
struct IOrientationFilter : virtual IFilter
{
    /*!
     * @brief Class desctructor.
     */
	~IOrientationFilter() override = default;

    /*!
     * @brief Get current orientation quaternion.
     * @return vector representing quaternion.
     */
    virtual quat::Quaternion get_orientation_quaternion() const = 0;

    /*!
     * @brief Get current gyroscope bias.
     * @return gyroscope bias vector.
     */
    virtual Vector3D get_gyro_bias() const = 0;
};

#endif /* INCLUDE_IORIENTATIONFILTER_H_ */
