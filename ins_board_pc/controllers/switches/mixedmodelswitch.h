#ifndef MIXEDMODELSWITCH_H
#define MIXEDMODELSWITCH_H

#include "controllers/switches/modelswitchbase.h"
#include "controllers/switches/positionmodelswitch.h"
#include "controllers/switches/orientationmodelswitch.h"

#include "filtering/filters/filtersfwd.h"

struct MixedModelSwitch : private ModelSwitchBase
{
    MixedModelSwitch(QComboBox * sw, std::shared_ptr<PositionModelSwitch> pos_sw, std::shared_ptr<OrientationModelSwitch> ori_sw);
    ~MixedModelSwitch() override = default;

    using ModelSwitchBase::enable;
    using ModelSwitchBase::disable;

    void switch_model(int cb_idx) override;

private:
    std::shared_ptr<PositionModelSwitch> pos_sw;
    std::shared_ptr<OrientationModelSwitch> ori_sw;

    FullEKF ekf_filter;
    FullUKF ukf_filter;
};

#endif // MIXEDMODELSWITCH_H
