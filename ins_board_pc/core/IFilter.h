#ifndef IFILTER_H
#define IFILTER_H

class FilterInput;

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
    virtual void do_reset()
    {
        int g = 0;
    }


private:
    virtual void do_step(const FilterInput & z)
    {
        int g = 0;
    }

};

#endif // IFILTER_H
