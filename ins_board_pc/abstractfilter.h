#ifndef ABSTRACTFILTER_H
#define ABSTRACTFILTER_H

#include "ublasaux.h"

#include <QDate>

class AbstractFilter
{
public:
    struct FilterInput
    {
        NumVector w;
        NumVector a;
        NumVector m;
        QDate day;
        NumVector geo;
        NumVector pos;
        NumVector v;
        double dt;
    };

    AbstractFilter()
    {
        reset_this();
    }

    virtual ~AbstractFilter() {}

    virtual void step(const FilterInput & z) = 0;

    NumVector get_state() const
    {
        return x;
    }

    virtual void reset()
    {
        reset_this();
    }

    bool is_initialized() const
    {
        return initialized;
    }

protected:
    virtual void update(const FilterInput & z) = 0;
    virtual void accumulate(const FilterInput & z) = 0;

    virtual void initialize(const FilterInput &)
    {
        initialized = true;
    }

    NumVector x;

private:
    bool initialized;

    void reset_this()
    {
        initialized = false;
    }
};

#endif // ABSTRACTFILTER_H
