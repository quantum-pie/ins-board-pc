#ifndef ORIENTATIONMODELSWITCH_H
#define ORIENTATIONMODELSWITCH_H

#include "filtering/filters/filtersfwd.h"
#include "controllers/direct/filtering/filteringcontrollersfwd.h"
#include "controllers/switches/singlemodelswitchbase.h"

class KalmanOrientationAttrController;

struct OrientationModelSwitch : private SingleModelSwitchBase<IOrientationFilter, IKalmanOrientationAttr>
{
    using base_type = SingleModelSwitchBase<IOrientationFilter, IKalmanOrientationAttr>;

    OrientationModelSwitch(QComboBox * sw, OrientationFilteringController & ori_ctrl, KalmanOrientationAttrController & attr_ctrl);

    using base_type::set_model;
    void switch_model(int cb_idx);

private:
    OrientationEKF ekf_filter;
    OrientationUKF ukf_filter;
};

#endif // ORIENTATIONMODELSWITCH_H
