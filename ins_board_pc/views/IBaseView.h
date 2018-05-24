#ifndef BASEVIEW_H
#define BASEVIEW_H

template<typename Provider>
struct IBaseView
{
    virtual void update(Provider * pvd) = 0;
};

struct IPositionProvider;
struct IOrientationProvider;

using IPositionView = IBaseView<IPositionProvider>;
using IOrientationView = IBaseView<IOrientationProvider>;

#endif // BASEORIENTATIONVIEW_H
