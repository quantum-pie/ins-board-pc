/*
 * IKalmanPositionFilterAttributes.h
 *
 *      Author: bigaw
 */

#ifndef INCLUDE_IKALMANPOSITIONFILTERATTRIBUTES_H_
#define INCLUDE_IKALMANPOSITIONFILTERATTRIBUTES_H_

#include "filtering/public_interfaces/IPositionFilter.h"
#include "core/IKalmanPositionAttr.h"

/*!
 * @brief Kalman position filter interface.
 */
struct IKalmanPositionFilter : IPositionFilter,
                               IKalmanPositionAttr
{
    ~IKalmanPositionFilter() override = default;
};

#endif /* INCLUDE_IKALMANPOSITIONFILTERATTRIBUTES_H_ */
