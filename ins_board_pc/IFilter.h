/*
 * IFilter.h
 *
 *      Author: Ermakov_P
 */

#ifndef INCLUDE_IFILTER_H_
#define INCLUDE_IFILTER_H_

class FilterInput;

/*!
 * @brief Filter interface.
 */
struct IFilter
{
    /*!
     * @brief Class desctructor.
     */
    virtual ~IFilter() = default;

    /*!
     * @brief Make step.
     * @param z filter input.
     */
    virtual void step(const FilterInput & z) = 0;

    /*!
     * @brief Reset filter to default state.
     */
    virtual void reset() = 0;
};

#endif /* INCLUDE_IFILTER_H_ */
