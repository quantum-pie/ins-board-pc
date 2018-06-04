#ifndef ORIENTATIONMODELSWITCH_H
#define ORIENTATIONMODELSWITCH_H

#include "filtering/filters/filtersfwd.h"
#include "controllers/direct/filtering/filteringcontrollersfwd.h"
#include "controllers/direct/attributes/kalmanorientationattrcontroller.h"
#include "controllers/switches/singlemodelswitchbase.h"

struct OrientationModelSwitch : private SingleModelSwitchBase<OrientationFilteringController, KalmanOrientationAttrController>
{
    using base_type = SingleModelSwitchBase<OrientationFilteringController, KalmanOrientationAttrController>;

    OrientationModelSwitch(QComboBox * sw, std::shared_ptr<OrientationFilteringController> ori_ctrl, std::unique_ptr<KalmanOrientationAttrController> attr_ctrl);

    using base_type::enable;
    using base_type::disable;
    using base_type::set_model;
    void switch_model(int cb_idx);

private:
    OrientationEKF ekf_filter;
    OrientationUKF ukf_filter;
};

#endif // ORIENTATIONMODELSWITCH_H
