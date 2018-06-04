#ifndef MIXEDMODELSWITCH_H
#define MIXEDMODELSWITCH_H

#include "controllers/switches/modelswitchbase.h"
#include "filtering/filters/filtersfwd.h"

class PositionModelSwitch;
class OrientationModelSwitch;

struct MixedModelSwitch : ModelSwitchBase
{
    MixedModelSwitch(QComboBox * sw, PositionModelSwitch & pos_sw, OrientationModelSwitch & ori_sw);

    using ModelSwitchBase::enable;
    using ModelSwitchBase::disable;
    void switch_model(int cb_idx);

private:
    PositionModelSwitch & pos_sw;
    OrientationModelSwitch & ori_sw;

    FullEKF ekf_filter;
    FullUKF ukf_filter;
};

#endif // MIXEDMODELSWITCH_H
