#include "rawviewmodel.h"

RawViewModel
Adapter<RawPacket, RawViewModel>::operator()(const RawPacket & packet)
{
    RawViewModel out { packet, Vector3D::Zero(), Vector3D::Zero(), Vector3D::Zero() };

    accel_ctrl.update(packet.a);
    gyro_ctrl.update(packet.w);
    magn_ctrl.update(packet.m);
    if(accel_ctrl.is_saturated())
    {
        out.accel_mean = accel_ctrl.get_mean();
        out.gyro_mean = gyro_ctrl.get_mean();
        out.magn_mean = magn_ctrl.get_mean();
    }

    return out;
}

void
Adapter<RawPacket, RawViewModel>::set_accumulator_capacity(std::size_t new_capacity)
{
    accel_ctrl.set_sampling(new_capacity);
    gyro_ctrl.set_sampling(new_capacity);
    magn_ctrl.set_sampling(new_capacity);
}
