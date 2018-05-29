#ifndef RAWCONTROLLER_H
#define RAWCONTROLLER_H

#include "views/IBaseView.h"
#include "controllers/direct/observablebase.h"
#include "receiver.h"
#include "packets.h"

#include <QObject>

class RawController : public QObject, public ObservableBase<IBaseView<RawPacket>>
{
    Q_OBJECT

public:
    RawController(const Receiver * receiver)
    {
        connect(receiver, SIGNAL(raw_packet_received(RawPacket)), this, SLOT(handle_input(RawPacket)));
    }

public slots:
    void handle_input(const RawPacket & z)
    {
        this->update_views(z);
    }
};

#endif // RAWCONTROLLER_H
