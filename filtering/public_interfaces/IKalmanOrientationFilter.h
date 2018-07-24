/*
 * IKalmanOrientationFilterAttributes.h
 *
 *      Author: bigaw
 */

#ifndef INCLUDE_IKALMANORIENTATIONFILTER_H_
#define INCLUDE_IKALMANORIENTATIONFILTER_H_

#include "filtering/public_interfaces/IOrientationFilter.h"
#include "core/IKalmanOrientationAttr.h"

/*!
 * @brief Kalman orientation filter interface.
 */
struct IKalmanOrientationFilter : IOrientationFilter,
                                  IKalmanOrientationAttr
{
    ~IKalmanOrientationFilter() override = default;
};

#endif /* INCLUDE_IKALMANORIENTATIONFILTER_H_ */
