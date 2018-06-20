#include "communication/udpreceiver.h"

UDPReceiver::UDPReceiver(const QString &ip, uint16_t port, processor_type proc)
    : processor{ proc }
{
    sock.bind(QHostAddress(ip), port);
    connect(&sock, SIGNAL(readyRead()), this, SLOT(read_datagrams()));
}

void UDPReceiver::set_processor(processor_type new_proc)
{
    processor = new_proc;
}

void UDPReceiver::read_datagrams()
{
    while(sock.hasPendingDatagrams())
    {
        QNetworkDatagram datagram = sock.receiveDatagram();
        processor(datagram.data());
    }
}
