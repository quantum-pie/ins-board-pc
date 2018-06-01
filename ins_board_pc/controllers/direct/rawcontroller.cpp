#include "controllers/direct/rawcontroller.h"
#include "receiver.h"
#include "packets.h"

#include <QPushButton>

RawController::RawController(const Receiver * receiver, const QPushButton * enable_btn)
{
    connect(receiver, SIGNAL(raw_packet_received(RawPacket)), this, SLOT(handle_input(RawPacket)));
    if(enable_btn)
    {
        set_running(enable_btn->isChecked());
        connect(enable_btn, SIGNAL(toggled(bool)), this, SLOT(set_running(bool)));
    }
}

void RawController::handle_input(const RawPacket & z)
{
    this->update_views(z);
}
