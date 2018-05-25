#ifndef ORIENTATIONMODELSWITCH_H
#define ORIENTATIONMODELSWITCH_H

#include "filtering/filters/generickalman.h"
#include "controllers/switches/singlemodelswitchbase.h"

class OrientationFilteringController;
class KalmanOrientationAttrController;

struct OrientationModelSwitch : SingleModelSwitchBase<IOrientationFilter, IKalmanOrientationAttr>
{
    using base_type = SingleModelSwitchBase<IOrientationFilter, IKalmanOrientationAttr>;

    OrientationModelSwitch(QComboBox * sw, OrientationFilteringController & ori_ctrl, KalmanOrientationAttrController & attr_ctrl);

    void switch_model(int cb_idx);

private:
    OrientationEKF ekf_filter;
    OrientationUKF ukf_filter;
};

#endif // ORIENTATIONMODELSWITCH_H
