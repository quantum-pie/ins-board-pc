#include "packets.h"
#include "magncalibrator.h"
#include "utils.h"

FilterInput parse_input(const RawPacket & in, const MagnCalibrator & magn_calib)
{
    Vector3D geo;
    geo <<  utils::degrees_to_radians(in.gps_data.geo[0]),
            utils::degrees_to_radians(in.gps_data.geo[1]),
            in.gps_data.geo[2];

    return {
        in.w.unaryExpr(&utils::degrees_to_radians),
        in.a,
        magn_calib.calibrate(in.m),
        geo,
        in.gps_data.pos,
        in.gps_data.vel,
        boost::gregorian::date(in.gps_data.time.year, in.gps_data.time.month, in.gps_data.time.day),
        in.et,
        in.gps_data.isnew && in.gps_data.fix
    };
}
