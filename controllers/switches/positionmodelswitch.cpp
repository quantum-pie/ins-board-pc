#include "controllers/switches/positionmodelswitch.h"

#include <QComboBox>

PositionModelSwitch::PositionModelSwitch(QComboBox * sw, std::shared_ptr<PositionFilteringController> pos_ctrl, std::unique_ptr<KalmanPositionAttrController> attr_ctrl)
    : base_type{ sw, pos_ctrl, std::move(attr_ctrl) }
{
    connect(sw, SIGNAL(activated(int)), this, SLOT(switch_model(int)));
}

void PositionModelSwitch::switch_model(int cb_idx)
{
    if(cb_idx == 0)
    {
        set_model(&ekf_filter);
    }
    else if(cb_idx == 1)
    {
        set_model(&ukf_filter);
    }
}
