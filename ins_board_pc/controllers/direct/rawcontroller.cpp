#include "controllers/direct/rawcontroller.h"
#include "receiver.h"
#include "packets.h"

RawController::RawController(const Receiver * receiver)
{
    connect(receiver, SIGNAL(raw_packet_received(RawPacket)), this, SLOT(handle_input(RawPacket)));
}

void RawController::handle_input(const RawPacket & z)
{
    this->update_views(z);
}
