#ifndef RECEIVER_H
#define RECEIVER_H

#include "communication/udpreceiver.h"
#include "packets.h"

#include <QObject>
#include <QString>

#include <cstdint>

class FilterInput;
class MagnCalibrator;

/*!
 * @brief The Receiver class.
 * This class is desogned to receive various inputs over UDP.
 */
class Receiver : public QObject
{
    Q_OBJECT

public:
    /*!
     * @brief Receiver constructor.
     * @param raw_pvd_ip IP address of the raw data provider.
     * @param raw_pvd_port port of the raw data provider.
     * @param flt_pvd_ip IP address of the filtered data provider.
     * @param flt_pvd_port port of the filtered data provider.
     * @param calibrator magnetic calibrator reference.
     */
    Receiver(const QString & raw_pvd_ip, uint16_t raw_pvd_port,
             const QString & flt_pvd_ip, uint16_t flt_pvd_port,
             const MagnCalibrator & calibrator);

    /*!
     * @brief Process raw data buffer.
     * @param data reference to raw data buffer.
     */
    void process_raw_data(const QByteArray & data);

    /*!
     * @brief Process filtered data buffer.
     * @param data reference to filtered data buffer.
     */
    void process_flt_data(const QByteArray & data);

    /*!
     * @brief Convert raw input data to filter input structure.
     * @param in raw input data instance.
     * @return filter input instance.
     */
    FilterInput parse_raw_data(const RawPacket & in);

signals:
    /*!
     * @brief Filter input sample received signal.
     * @param z Filter input sample.
     */
    void raw_sample_received(const FilterInput & z);

    /*!
     * @brief Raw data packet received signal.
     * @param z raw data packet.
     */
    void raw_packet_received(const RawPacket & z);

    /*!
     * @brief Filtered data packet received signal.
     * @param z filtered data packet.
     */
    void filtered_packet_received(const FilteredPacket & z);

private:
    UDPReceiver raw_recv;
    UDPReceiver flt_recv;

    Timestamp old_timestamp;

    const MagnCalibrator & calibrator;
};

#endif // RECEIVER_H
