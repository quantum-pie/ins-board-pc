#include "udpsender.h"

UDPSender::UDPSender(const QString &ip, uint16_t port) : server_ip{ ip }, port{ port }
{}

void UDPSender::write_data(const char * data, int64_t size)
{
    sock.writeDatagram(data, size, server_ip, port);
}
