#ifndef ISIMPOSITIONFILTER_H
#define ISIMPOSITIONFILTER_H

#include "filtering/public_interfaces/IPositionFilter.h"
#include "core/ISimPositionAttr.h"

struct ISimPositionFilter : IPositionFilter,
                            ISimPositionAttr
{
    ~ISimPositionFilter() override = default;
};

#endif // ISIMPOSITIONFILTER_H
