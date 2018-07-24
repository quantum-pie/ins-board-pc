#ifndef IFILTER_H
#define IFILTER_H

class FilterInput;

/*!
 * @brief The Filter interface.
 */
struct IFilter
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

protected:
    /*!
     * @brief Filter reset implementation.
     */
    virtual void do_reset() = 0;

private:
    virtual void do_step(const FilterInput & z) = 0;
};

#endif // IFILTER_H
