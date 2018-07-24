#ifndef UDPRECEIVER_H
#define UDPRECEIVER_H

#include <QtNetwork>
#include <QObject>

#include <functional>

/*!
 * @brief The UDPReceiver class
 * This is the wrapper of input UDP socket.
 */
class UDPReceiver : public QObject
{
    Q_OBJECT

public:
    //! Processing function type alias.
    using processor_type = std::function<void(const QByteArray & data)>;

    /*!
     * @brief UDPReceiver constructor.
     * @param ip Client IP address.
     * @param port Client port.
     * @param proc processing functor.
     */
    UDPReceiver(const QString & ip, uint16_t port, processor_type proc);

    /*!
     * @brief Set processing functor.
     * @param new_proc new processor.
     */
    void set_processor(processor_type new_proc);

public slots:
    /*!
     * @brief Read pending datagrams from socket.
     */
    void read_datagrams();

private:
    QUdpSocket sock;
    processor_type processor;
};

#endif // UDPRECEIVER_H
