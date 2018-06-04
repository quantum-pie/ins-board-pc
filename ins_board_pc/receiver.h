#ifndef RECEIVER_H
#define RECEIVER_H

#include "udpreceiver.h"

#include <QObject>
#include <QString>

#include <cstdint>

class FilterInput;
class RawPacket;
class FilteredPacket;
class MagnCalibrator;

class Receiver : public QObject
{
    Q_OBJECT

public:
    Receiver(const QString & raw_pvd_ip, uint16_t raw_pvd_port,
             const QString & flt_pvd_ip, uint16_t flt_pvd_port,
             const MagnCalibrator & calibrator);

    void process_raw_data(const QByteArray & data);
    void process_flt_data(const QByteArray & data);

    /*!
     * @brief Convert raw input data to filter input structure.
     * @param in raw input data instance.
     * @return filter input instance.
     */
    FilterInput parse_raw_data(const RawPacket & in);

signals:
    void raw_sample_received(const FilterInput & z);
    void raw_packet_received(const RawPacket & z);
    void filtered_packet_received(const FilteredPacket & z);

private:
    UDPReceiver raw_recv;
    UDPReceiver flt_recv;

    const MagnCalibrator & calibrator;
};

#endif // RECEIVER_H
