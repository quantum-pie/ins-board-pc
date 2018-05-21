#ifndef IFILTERCORE_H
#define IFILTERCORE_H

class FilterInput;

struct IFilterCore
{
public:
    /*!
     * @brief Class desctructor.
     */
    virtual ~IFilterCore() = default;

    virtual void do_step(const FilterInput & z) = 0;
    virtual void do_reset() = 0;
};

#endif // IFILTERCORE_H
