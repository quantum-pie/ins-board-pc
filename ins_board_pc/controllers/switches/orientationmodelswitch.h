#ifndef ORIENTATIONMODELSWITCH_H
#define ORIENTATIONMODELSWITCH_H

#include "filtering/filters/filtersfwd.h"
#include "controllers/direct/filtering/filteringcontrollersfwd.h"
#include "controllers/direct/attributes/kalmanorientationattrcontroller.h"
#include "controllers/switches/singlemodelswitchbase.h"

/*!
 * @brief The OrientationModelSwitch struct
 * This is the orientation model switch.
 */
struct OrientationModelSwitch : SingleModelSwitchBase<OrientationFilteringController, KalmanOrientationAttrController>
{
    //! Base type alias.
    using base_type = SingleModelSwitchBase<OrientationFilteringController, KalmanOrientationAttrController>;

    /*!
     * @brief OrientationModelSwitch constructor.
     * @param sw Switch widget.
     * @param ori_ctrl Orientation filtering controller.
     * @param attr_ctrl Orientation filter attributes controller.
     */
    OrientationModelSwitch(QComboBox * sw, std::shared_ptr<OrientationFilteringController> ori_ctrl, std::unique_ptr<KalmanOrientationAttrController> attr_ctrl);

    ~OrientationModelSwitch() override = default;

    void switch_model(int cb_idx) override;

private:
    OrientationEKF ekf_filter;
    OrientationUKF ukf_filter;
};

#endif // ORIENTATIONMODELSWITCH_H
