#ifndef MIXEDMODELSWITCH_H
#define MIXEDMODELSWITCH_H

#include "controllers/switches/modelswitchbase.h"
#include "filtering/filters/filtersfwd.h"

class PositionModelSwitch;
class OrientationModelSwitch;

struct MixedModelSwitch : ModelSwitchBase
{
    MixedModelSwitch(QComboBox * sw, std::shared_ptr<PositionModelSwitch> pos_sw, std::shared_ptr<OrientationModelSwitch> ori_sw);

    using ModelSwitchBase::enable;
    using ModelSwitchBase::disable;
    void switch_model(int cb_idx);

private:
    std::shared_ptr<PositionModelSwitch> pos_sw;
    std::shared_ptr<OrientationModelSwitch> ori_sw;

    FullEKF ekf_filter;
    FullUKF ukf_filter;
};

#endif // MIXEDMODELSWITCH_H
