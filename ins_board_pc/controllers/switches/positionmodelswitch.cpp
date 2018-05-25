#include "controllers/switches/positionmodelswitch.h"
#include "controllers/direct/filtering/filteringcontrollersfwd.h"
#include "controllers/direct/attributes/kalmanpositionattrcontroller.h"

#include <QComboBox>

PositionModelSwitch::PositionModelSwitch(QComboBox * sw, PositionFilteringController & pos_ctrl, KalmanPositionAttrController & attr_ctrl)
    : base_type{ sw, pos_ctrl, attr_ctrl }
{
    connect(sw, SIGNAL(currentIndexChanged(int)), this, SLOT(switch_model(int)));
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
