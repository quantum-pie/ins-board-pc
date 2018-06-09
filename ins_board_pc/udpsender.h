#ifndef UDPSENDER_H
#define UDPSENDER_H

#include <QtNetwork>

struct UDPSender
{
    UDPSender(const QString & ip, uint16_t port);
    void write_data(const char * data, int64_t size);

private:
    QUdpSocket sock;
    QHostAddress server_ip;
    uint16_t port;
};

#endif // UDPSENDER_H
