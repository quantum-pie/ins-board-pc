#include "adapters/orientationfilteringviewmodel.h"
#include "quatutils.h"
#include "utils.h"

using namespace quat;

OrientationFilteringViewModel
Adapter<IOrientationFilter, OrientationFilteringViewModel>::operator()(const IOrientationFilter & filter)
{
    auto q = filter.get_orientation_quaternion();
    auto rpy = q.rpy();
    auto qf = static_cast<vector_form>(q).cast<float>();
    QQuaternion qq{ qf[0], qf[1], qf[2], qf[3] };

    OrientationFilteringViewModel out { qq, rpy, Vector3D::Zero(), Vector3D::Zero() };

    std_ctrl.update(rpy);
    if(std_ctrl.is_saturated())
    {
        out.rpy_std = std_ctrl.get_std();
        out.rpy_mean = std_ctrl.get_mean();
    }

    return out;
}

void
Adapter<IOrientationFilter, OrientationFilteringViewModel>::set_accumulator_capacity(std::size_t new_capacity)
{
    std_ctrl.set_sampling(new_capacity);
}

OrientationFilteringViewModel
Adapter<FilteredPacket, OrientationFilteringViewModel>::operator()(const FilteredPacket & packet)
{
    double roll = utils::fixed_to_angle(packet.roll);
    double pitch = utils::fixed_to_angle(packet.pitch);
    double yaw = utils::fixed_to_angle(packet.heading);
    auto q = static_cast<vector_form>(z_rotator(-yaw) * x_rotator(pitch) * y_rotator(roll)).cast<float>();
    QQuaternion qq{ q[0], q[1], q[2], q[3] };

    Vector3D rpy;
    rpy << roll, pitch, yaw;

    OrientationFilteringViewModel out { qq, rpy, Vector3D::Zero(), Vector3D::Zero() };

    std_ctrl.update(rpy);
    if(std_ctrl.is_saturated())
    {
        out.rpy_std = std_ctrl.get_std();
        out.rpy_mean = std_ctrl.get_mean();
    }

    return out;
}

void
Adapter<FilteredPacket, OrientationFilteringViewModel>::set_accumulator_capacity(std::size_t new_capacity)
{
    std_ctrl.set_sampling(new_capacity);
}
