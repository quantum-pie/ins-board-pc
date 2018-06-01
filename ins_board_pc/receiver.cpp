#include "receiver.h"

Receiver::Receiver(const QString &raw_pvd_ip, uint16_t raw_pvd_port, const QString &flt_pvd_ip, uint16_t flt_pvd_port)
{
    raw_socket.bind(QHostAddress(raw_pvd_ip), raw_pvd_port);
    connect(&raw_socket, SIGNAL(readyRead()), this, SLOT(read_raw_datagrams()));

    flt_socket.bind(QHostAddress(flt_pvd_ip), flt_pvd_port);
    connect(&flt_socket, SIGNAL(readyRead()), this, SLOT(read_flt_datagrams()));
}

void Receiver::read_raw_datagrams()
{
    while(raw_socket.hasPendingDatagrams())
    {
        QNetworkDatagram datagram = raw_socket.receiveDatagram();
        process_raw_data(datagram.data());
    }
}

void Receiver::read_flt_datagrams()
{
    while(flt_socket.hasPendingDatagrams())
    {
        QNetworkDatagram datagram = flt_socket.receiveDatagram();
        process_flt_data(datagram.data());
    }
}
