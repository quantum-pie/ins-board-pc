#ifndef BASEVIEW_H
#define BASEVIEW_H

class FilterInput;

#include <QDebug>

template<typename Model>
struct IBaseView
{
    using ViewModel = Model;

    virtual ~IBaseView() = default;
    virtual void update(const ViewModel & pvd) = 0;
    virtual void clear() = 0;
};

struct IPositionProvider;
struct IOrientationProvider;
struct RawPacket;
struct FilteredPacket;
class MagnCalibrator;

using IPositionView = IBaseView<IPositionProvider>;
using IOrientationView = IBaseView<IOrientationProvider>;
using IRawView = IBaseView<RawPacket>;
using IRemoteView = IBaseView<FilteredPacket>;
using ICalibrationView = IBaseView<MagnCalibrator>;

#endif // BASEORIENTATIONVIEW_H
