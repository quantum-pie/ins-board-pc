/*
 * IComplementOrientationFilterAttributes.h
 *
 *      Author: bigaw
 */

#ifndef INCLUDE_ICOMPLEMENTORIENTATIONFILTER_H_
#define INCLUDE_ICOMPLEMENTORIENTATIONFILTER_H_

#include "filtering/public_interfaces/IOrientationFilter.h"
#include "core/IComplementOrientationAttr.h"

/*!
 * @brief Complementary orientation filter interface.
 */
struct IComplementOrientationFilter : IOrientationFilter,
                                      IComplementOrientationAttr
{
    ~IComplementOrientationFilter() override = default;
};

#endif /* INCLUDE_ICOMPLEMENTORIENTATIONFILTER_H_ */
