#include "adapters/positionfilteringviewmodel.h"
#include "utils.h"
#include "geometry.h"

PositionFilteringViewModel
Adapter<IPositionFilter, PositionFilteringViewModel>::operator()(const IPositionFilter & filter)
{
    PositionFilteringViewModel out {filter.get_ellipsoid(), filter.get_cartesian(), 0, 0};
    speed_accum.update(filter.get_velocity());
    if(speed_accum.is_saturated())
    {
        Vector3D geo = geom::cartesian_to_geodetic(filter.get_cartesian(), filter.get_ellipsoid());
        out.track_angle = geom::track_angle(speed_accum.get_mean(), geo);
        out.ground_speed = utils::ms_to_knots(geom::ground_speed(speed_accum.get_mean(), geo));
    }

    return out;
}

void
Adapter<IPositionFilter, PositionFilteringViewModel>::set_accumulator_capacity(std::size_t new_capacity)
{
    speed_accum.set_sampling(new_capacity);
}

PositionFilteringViewModel
Adapter<FilteredPacket, PositionFilteringViewModel>::operator()(const FilteredPacket & packet)
{
    Vector3D pos;
    pos << packet.ecef_x, packet.ecef_y, packet.ecef_z;

    return {Ellipsoid::WGS84, pos, utils::fixed_to_double(packet.gspeed), utils::fixed_to_angle(packet.track)};
}

void
Adapter<FilteredPacket, PositionFilteringViewModel>::set_accumulator_capacity(std::size_t) {}

