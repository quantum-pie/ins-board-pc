#ifndef IFILTERINGMODEL_H
#define IFILTERINGMODEL_H

#include "models/filteringmodelbase.h"

class FilterInput;

template<typename FilterInterface>
struct IFilteringModel : FilteringModelBase, FilterInterface
{
    void step(const FilterInput & z)
    {
        FilterInterface::step(z);
        emit refreshed();
    }
};

#endif // IFILTERINGMODEL_H
