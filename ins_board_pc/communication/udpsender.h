#ifndef UDPSENDER_H
#define UDPSENDER_H

#include <QtNetwork>

/*!
 * @brief The UDPSender struct
 * This is the wrapper of output UDP socket.
 */
struct UDPSender
{
    /*!
     * @brief UDPSender constructor.
     * @param ip Server IP address.
     * @param port Server port.
     */
    UDPSender(const QString & ip, uint16_t port);

    /*!
     * @brief write_data Write data to socket.
     * @param data pointer to output data.
     * @param size size of output data.
     */
    void write_data(const char * data, int64_t size);

private:
    QUdpSocket sock;
    QHostAddress server_ip;
    uint16_t port;
};

#endif // UDPSENDER_H
