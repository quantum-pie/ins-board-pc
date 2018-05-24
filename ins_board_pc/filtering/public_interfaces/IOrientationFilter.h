/*
 * IOrientationProvider.h
 *
 *      Author: bigaw
 */

#ifndef INCLUDE_IORIENTATIONFILTER_H_
#define INCLUDE_IORIENTATIONFILTER_H_

#include "core/IFilter.h"
#include "core/IOrientationProvider.h"

/*!
 * @brief Orientation filter interface.
 */
struct IOrientationFilter : IFilter, IOrientationProvider
{
    /*!
     * @brief Class desctructor.
     */
	~IOrientationFilter() override = default;
};

#endif /* INCLUDE_IORIENTATIONFILTER_H_ */
