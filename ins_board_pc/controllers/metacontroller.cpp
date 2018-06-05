#include "controllers/metacontroller.h"
#include "controllers/switches/mixedmodelswitch.h"
#include "controllers/switches/positionmodelswitch.h"
#include "controllers/switches/orientationmodelswitch.h"
#include "controllers/direct/filtering/filteringcontrollersfwd.h"

#include <QComboBox>

MetaController::MetaController(QComboBox * meta_cb, std::shared_ptr<PositionModelSwitch> pos_sw, std::shared_ptr<OrientationModelSwitch> ori_sw,
               std::unique_ptr<MixedModelSwitch> mix_sw, std::shared_ptr<PositionFilteringController> pos_ctrl)
    : pos_sw{ pos_sw }, ori_sw{ ori_sw }, mix_sw{ std::move(mix_sw) }, pos_ctrl{ pos_ctrl }
{
    configure_control(meta_cb->currentIndex());
    connect(meta_cb, SIGNAL(currentIndexChanged(int)), this, SLOT(configure_control(int)));
}

void MetaController::configure_control(int cb_idx)
{
    if(cb_idx == 0)
    {
        pos_ctrl->enable_filtering();
        pos_sw->enable();
        ori_sw->enable();
        mix_sw->disable();
    }
    else if(cb_idx == 1)
    {
        pos_ctrl->disable_filtering();
        pos_sw->disable();
        ori_sw->disable();
        mix_sw->enable();
    }
}
