#include "controllers/switches/orientationmodelswitch.h"

#include <QComboBox>

OrientationModelSwitch::OrientationModelSwitch(QComboBox * sw, OrientationFilteringController & ori_ctrl, KalmanOrientationAttrController & attr_ctrl)
    : base_type{ sw, ori_ctrl, attr_ctrl }
{
    connect(sw, SIGNAL(currentIndexChanged(int)), this, SLOT(switch_model(int)));
}

void OrientationModelSwitch::switch_model(int cb_idx)
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
