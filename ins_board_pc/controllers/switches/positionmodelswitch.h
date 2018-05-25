#ifndef POSITIONMODELSWITCH_H
#define POSITIONMODELSWITCH_H

#include "filtering/filters/generickalman.h"
#include "controllers/switches/singlemodelswitchbase.h"

class PositionFilteringController;
class KalmanPositionAttrController;

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
