#include "controllers/switches/mixedmodelswitch.h"
#include "controllers/switches/positionmodelswitch.h"
#include "controllers/switches/orientationmodelswitch.h"

#include <QComboBox>

MixedModelSwitch::MixedModelSwitch(QComboBox * sw, PositionModelSwitch & pos_sw, OrientationModelSwitch & ori_sw)
    : ModelSwitchBase{ sw }, pos_sw{ pos_sw }, ori_sw{ ori_sw }
{
    connect(sw, SIGNAL(currentIndexChanged(int)), this, SLOT(switch_model(int)));
}

void MixedModelSwitch::switch_model(int cb_idx)
{
    if(cb_idx == 0)
    {
        pos_sw.set_model(&ekf_filter);
        ori_sw.set_model(&ekf_filter);
    }
    else if(cb_idx == 1)
    {
        pos_sw.set_model(&ukf_filter);
        ori_sw.set_model(&ukf_filter);
    }
}