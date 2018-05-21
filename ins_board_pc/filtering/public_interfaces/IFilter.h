/*
 * IFilter.h
 *
 *      Author: Ermakov_P
 */

#ifndef INCLUDE_IFILTER_H_
#define INCLUDE_IFILTER_H_

#include "core/IFilterCore.h"

/*!
 * @brief Filter interface.
 */
struct IFilter : private IFilterCore
{
    /*!
     * @brief Make step.
     * @param z filter input.
     */
    void step(const FilterInput & z)
    {
        do_step(z);
    }

    /*!
     * @brief Reset filter to default state.
     */
    void reset()
    {
        do_reset();
    }

    /*!
     * @brief Class desctructor.
     */
    virtual ~IFilter() = default;
};

#endif /* INCLUDE_IFILTER_H_ */
