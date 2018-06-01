#ifndef POSITIONMODELSWITCH_H
#define POSITIONMODELSWITCH_H

#include "filtering/filters/filtersfwd.h"
#include "controllers/direct/filtering/filteringcontrollersfwd.h"
#include "controllers/switches/singlemodelswitchbase.h"

class KalmanPositionAttrController;

struct PositionModelSwitch : private SingleModelSwitchBase<IPositionFilter, IKalmanPositionAttr>
{
    using base_type = SingleModelSwitchBase<IPositionFilter, IKalmanPositionAttr>;

    PositionModelSwitch(QComboBox * sw, PositionFilteringController & pos_ctrl, KalmanPositionAttrController & attr_ctrl);

    using base_type::set_model;
    void switch_model(int cb_idx);

private:
    PositionEKF ekf_filter;
    PositionUKF ukf_filter;
};

#endif // POSITIONMODELSWITCH_H
