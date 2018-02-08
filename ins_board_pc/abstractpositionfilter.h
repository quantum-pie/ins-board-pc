#ifndef ABSTRACTPOSITIONFILTER_H
#define ABSTRACTPOSITIONFILTER_H

#include "abstractfilter.h"

class AbstractPositionFilter : public virtual AbstractFilter
{
public:
    AbstractPositionFilter() : AbstractFilter()
    {
        reset_this();
    }

    ~AbstractPositionFilter() override {}

    void reset() override
    {
        AbstractFilter::reset();
        reset_this();
    }

    virtual NumVector get_position() = 0;
    virtual NumVector get_velocity() = 0;
    virtual NumVector get_acceleration() = 0;

    virtual void get_geodetic(double & lat, double & lon, double & alt) = 0;

protected:
    void accumulate(const FilterInput &) override {}

private:
    void reset_this()
    {

    }
};

#endif // ABSTRACTPOSITIONFILTER_H