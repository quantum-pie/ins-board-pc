#ifndef ORIENTATIONFILTERINGVIEWMODEL_H
#define ORIENTATIONFILTERINGVIEWMODEL_H

#include "adapters/adapter.h"
#include "eigenaux.h"
#include "filtering/public_interfaces/IOrientationFilter.h"
#include "packets.h"
#include "qualitycontrol.h"

#include <QQuaternion>

struct OrientationFilteringViewModel
{
    QQuaternion q;
    Vector3D rpy;
    Vector3D rpy_std;
    Vector3D rpy_mean;
};

template<>
struct Adapter<IOrientationFilter, OrientationFilteringViewModel>
{
    OrientationFilteringViewModel operator()(const IOrientationFilter & filter);
    void set_accumulator_capacity(std::size_t new_capacity);

private:
    QualityControl<Vector3D> std_ctrl;
};

template<>
struct Adapter<FilteredPacket, OrientationFilteringViewModel>
{
    OrientationFilteringViewModel operator()(const FilteredPacket & packet);
    void set_accumulator_capacity(std::size_t new_capacity);

private:
    QualityControl<Vector3D> std_ctrl;
};

#endif // ORIENTATIONFILTERINGVIEWMODEL_H
