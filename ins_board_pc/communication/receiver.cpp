#include "communication/receiver.h"
#include "packets.h"
#include "magncalibrator.h"
#include "utils.h"

#include <QDataStream>

Receiver::Receiver(const QString &raw_pvd_ip, uint16_t raw_pvd_port,
                   const QString &flt_pvd_ip, uint16_t flt_pvd_port,
                   const MagnCalibrator &calibrator)
    : raw_recv( raw_pvd_ip, raw_pvd_port, [this](const QByteArray &ar){ process_raw_data(ar); } ),
      flt_recv( flt_pvd_ip, flt_pvd_port, [this](const QByteArray &ar){ process_flt_data(ar); } ),
      calibrator{ calibrator }
{}

Receiver::~Receiver()
{
    raw_recv.set_processor(nullptr);
    flt_recv.set_processor(nullptr);
}

void Receiver::process_raw_data(const QByteArray & data)
{
    QDataStream ds(data);
    ds.setByteOrder(QDataStream::LittleEndian);

    RawPacket raw_z;
    ds >> raw_z.pkt_number
            >> raw_z.gps_data.time.year >> raw_z.gps_data.time.month >> raw_z.gps_data.time.day
            >> raw_z.gps_data.time.hour >> raw_z.gps_data.time.minute
            >> raw_z.gps_data.time.second >> raw_z.gps_data.time.ssecond
            >> raw_z.gps_data.geo[0] >> raw_z.gps_data.geo[1] >> raw_z.gps_data.geo[2]
            >> raw_z.gps_data.msl_altitude
            >> raw_z.gps_data.pos[0] >> raw_z.gps_data.pos[1] >> raw_z.gps_data.pos[2]
            >> raw_z.gps_data.vel[0] >> raw_z.gps_data.vel[1] >> raw_z.gps_data.vel[2]
            >> raw_z.gps_data.fix >> raw_z.gps_data.isnew
            >> raw_z.w[0] >> raw_z.w[1] >> raw_z.w[2]
            >> raw_z.a[0] >> raw_z.a[1] >> raw_z.a[2]
            >> raw_z.m[0] >> raw_z.m[1] >> raw_z.m[2]
            >> raw_z.et;

    emit raw_packet_received(raw_z);
    emit raw_sample_received(parse_raw_data(raw_z));
}

void Receiver::process_flt_data(const QByteArray & data)
{
    QDataStream ds(data);
    ds.setByteOrder(QDataStream::LittleEndian);

    FilteredPacket f_z;

    ds >> f_z.status
            >> f_z.ecef_x >> f_z.ecef_y >> f_z.ecef_z
            >> f_z.gps_lat >> f_z.gps_lon >> f_z.gps_alt
            >> f_z.gspeed >> f_z.heading >> f_z.pitch
            >> f_z.roll >> f_z.track >> f_z.pkt_number;

    emit filtered_packet_received(f_z);
}

FilterInput Receiver::parse_raw_data(const RawPacket & in)
{
    Vector3D geo;
    geo <<  utils::degrees_to_radians(in.gps_data.geo[0]),
            utils::degrees_to_radians(in.gps_data.geo[1]),
            in.gps_data.geo[2];

    return {
        in.w.unaryExpr(&utils::degrees_to_radians),
        in.a,
        calibrator.calibrate(in.m),
        geo,
        in.gps_data.pos,
        in.gps_data.vel,
        boost::gregorian::date(in.gps_data.time.year, in.gps_data.time.month, in.gps_data.time.day),
        in.et,
        in.gps_data.isnew && in.gps_data.fix
    };
}
