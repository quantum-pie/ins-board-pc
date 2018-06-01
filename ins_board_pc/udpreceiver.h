#ifndef UDPRECEIVER_H
#define UDPRECEIVER_H

#include <QtNetwork>

#include <functional>

class UDPReceiver
{
public:
    using processor_type = std::function<void(const QByteArray & data)>;

    UDPReceiver(const QString & ip, uint16_t port, processor_type proc);

private:
    QUdpSocket sock;
};

#endif // UDPRECEIVER_H
