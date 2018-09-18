#ifndef RAWVIEWMODEL_H
#define RAWVIEWMODEL_H

#include "adapters/adapter.h"
#include "packets.h"
#include "qualitycontrol.h"

struct RawViewModel
{
    RawPacket packet;
    Vector3D accel_mean;
    Vector3D gyro_mean;
    Vector3D magn_mean;
};

template<>
struct Adapter<RawPacket, RawViewModel>
{
    RawViewModel operator()(const RawPacket & packet);

    void set_accumulator_capacity(std::size_t new_capacity);
private:
    QualityControl<Vector3D> accel_ctrl;
    QualityControl<Vector3D> gyro_ctrl;
    QualityControl<Vector3D> magn_ctrl;
};

#endif // RAWVIEWMODEL_H
