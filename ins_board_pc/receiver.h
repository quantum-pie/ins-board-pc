#ifndef RECEIVER_H
#define RECEIVER_H

#include <QtNetwork>
#include <QObject>
#include <QDataStream>
#include <QString>

#include <cstdint>

class FilterInput;
class RawPacket;
class FilteredPacket;

class Receiver : public QObject
{
    Q_OBJECT

public:
    Receiver(const QString & raw_pvd_ip, uint16_t raw_pvd_port,
             const QString & flt_pvd_ip, uint16_t flt_pvd_port);

public slots:
    void read_raw_datagrams();
    void read_flt_datagrams();

signals:
    void raw_sample_received(const FilterInput & z);
    void raw_packet_received(const RawPacket & z);
    void filtered_packet_received(const FilteredPacket & z);

private:
    QUdpSocket raw_socket;
    QUdpSocket flt_socket;
};

#endif // RECEIVER_H
