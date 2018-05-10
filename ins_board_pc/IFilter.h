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
public:
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

private:
    virtual void do_step(const FilterInput & z) = 0;
    virtual void do_reset() = 0;
};

#endif /* INCLUDE_IFILTER_H_ */
