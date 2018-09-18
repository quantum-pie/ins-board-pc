#include "controllers/direct/rawcontroller.h"
#include "communication/receiver.h"
#include "packets.h"

#include <QPushButton>

RawController::RawController(const QPushButton * enable_btn)
{
    if(enable_btn)
    {
        set_running(enable_btn->isChecked());
        connect(enable_btn, SIGNAL(toggled(bool)), this, SLOT(handle_enable(bool)));
    }
}

void RawController::handle_input(const RawPacket & z)
{
    if(is_running())
    {
        update_views(mvm_adapter(z));
    }
}

void RawController::handle_enable(bool en)
{
    set_running(en);
    if(en)
    {
        clear_views();
    }
}
