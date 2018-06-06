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
class MagnCalibrator;

template<typename Model>
struct FilteringViewModel
{
    const Model & pvd_ref;
    const FilterInput & raw_ref;
};

using PositionFilteringViewModel = FilteringViewModel<IPositionProvider>;
using OrientationFilteringViewModel = FilteringViewModel<IOrientationProvider>;

using IPositionView = IBaseView<PositionFilteringViewModel>;
using IOrientationView = IBaseView<OrientationFilteringViewModel>;
using IRawView = IBaseView<RawPacket>;
using ICalibrationView = IBaseView<MagnCalibrator>;

#endif // BASEORIENTATIONVIEW_H
