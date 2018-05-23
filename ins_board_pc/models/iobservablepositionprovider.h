#ifndef IOBSERVABLEPOSITIONPROVIDER_H
#define IOBSERVABLEPOSITIONPROVIDER_H

#include "core/IPositionProviderCore.h"

#include <QtPlugin>

struct IObservablePositionProvider : private IPositionProviderCore
{
    virtual ~IObservablePositionProvider() = default;

signals:
    virtual void position_updated() = 0;
}

Q_DECLARE_INTERFACE(IObservablePositionProvider, "IObservablePositionProvider")

#endif // IOBSERVABLEPOSITIONPROVIDER_H
