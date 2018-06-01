#include "udpreceiver.h"

UDPReceiver::UDPReceiver(const QString &ip, uint16_t port, processor_type proc)
{
    sock.bind(QHostAddress(ip), port);
    connect(&raw_socket, SIGNAL(readyRead()), this, SLOT(read_raw_datagrams()));
}
