#ifndef POSITIONMODELSWITCH_H
#define POSITIONMODELSWITCH_H

#include "filtering/filters/filtersfwd.h"
#include "controllers/direct/filtering/filteringcontrollersfwd.h"
#include "controllers/switches/singlemodelswitchbase.h"

struct PositionModelSwitch : SingleModelSwitchBase<IPositionFilter, IKalmanPositionAttr>
{
    using base_type = SingleModelSwitchBase<IPositionFilter, IKalmanPositionAttr>;

    PositionModelSwitch(QComboBox * sw, PositionFilteringController & pos_ctrl, KalmanPositionAttrController & attr_ctrl);

    void switch_model(int cb_idx);

private:
    PositionEKF ekf_filter;
    PositionUKF ukf_filter;
};

#endif // POSITIONMODELSWITCH_H
