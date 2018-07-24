#ifndef POSITIONMODELSWITCH_H
#define POSITIONMODELSWITCH_H

#include "filtering/filters/filtersfwd.h"
#include "controllers/direct/filtering/filteringcontrollersfwd.h"
#include "controllers/direct/attributes/kalmanpositionattrcontroller.h"
#include "controllers/switches/singlemodelswitchbase.h"

/*!
 * @brief The PositionModelSwitch struct
 * This is the position filtering model switch.
 */
struct PositionModelSwitch : SingleModelSwitchBase<PositionFilteringController, KalmanPositionAttrController>
{
    //! Base type alias.
    using base_type = SingleModelSwitchBase<PositionFilteringController, KalmanPositionAttrController>;

    /*!
     * @brief PositionModelSwitch constructor.
     * @param sw Switch widget.
     * @param pos_ctrl Position filtering controller.
     * @param attr_ctrl Position filter attributes controller.
     */
    PositionModelSwitch(QComboBox * sw, std::shared_ptr<PositionFilteringController> pos_ctrl, std::unique_ptr<KalmanPositionAttrController> attr_ctrl);
    ~PositionModelSwitch() override = default;

    void switch_model(int cb_idx) override;

private:
    PositionEKF ekf_filter;
    PositionUKF ukf_filter;
};

#endif // POSITIONMODELSWITCH_H
