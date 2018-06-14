#ifndef UDPRECEIVER_H
#define UDPRECEIVER_H

#include <QtNetwork>
#include <QObject>

#include <functional>

class UDPReceiver : public QObject
{
    Q_OBJECT

public:
    using processor_type = std::function<void(const QByteArray & data)>;

    UDPReceiver(const QString & ip, uint16_t port, processor_type proc);
    void set_processor(processor_type new_proc);

public slots:
    void read_datagrams();

private:
    QUdpSocket sock;
    processor_type processor;
};

#endif // UDPRECEIVER_H
