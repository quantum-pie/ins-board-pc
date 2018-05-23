#ifndef IOBSERVABLEORIENTATIONPROVIDER_H
#define IOBSERVABLEORIENTATIONPROVIDER_H

#include "core/IOrientationProviderCore.h"

#include <QtPlugin>

struct IObservableOrientationProvider : private IOrientationProviderCore
{
    virtual ~IObservableOrientationProvider() = default;

signals:
    virtual void orientation_updated() = 0;
}

Q_DECLARE_INTERFACE(IObservableOrientationProvider, "IObservableOrientationProvider")

#endif // IOBSERVABLEORIENTATIONPROVIDER_H
