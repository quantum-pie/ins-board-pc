#ifndef MIXEDMODELSWITCH_H
#define MIXEDMODELSWITCH_H

#include "controllers/switches/modelswitchbase.h"
#include "controllers/switches/positionmodelswitch.h"
#include "controllers/switches/orientationmodelswitch.h"

#include "filtering/filters/filtersfwd.h"

/*!
 * @brief The MixedModelSwitch struct
 * This class encapsulates the switch between mixed filtering models.
 */
struct MixedModelSwitch : private ModelSwitchBase
{
    /*!
     * @brief MixedModelSwitch constructor.
     * @param sw Switch widget.
     * @param pos_sw Position model switch.
     * @param ori_sw Orientation model switch.
     */
    MixedModelSwitch(QComboBox * sw, std::shared_ptr<PositionModelSwitch> pos_sw, std::shared_ptr<OrientationModelSwitch> ori_sw);

    ~MixedModelSwitch() override = default;

    //! Bring switch enable to scope.
    using ModelSwitchBase::enable;

    //! Bring switch disable to scope.
    using ModelSwitchBase::disable;

    void switch_model(int cb_idx) override;

private:
    std::shared_ptr<PositionModelSwitch> pos_sw;
    std::shared_ptr<OrientationModelSwitch> ori_sw;

    FullEKF ekf_filter;
    FullUKF ukf_filter;
};

#endif // MIXEDMODELSWITCH_H
