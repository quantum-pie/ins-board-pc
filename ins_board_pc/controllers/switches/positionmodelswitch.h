#ifndef POSITIONMODELSWITCH_H
#define POSITIONMODELSWITCH_H

#include "filtering/filters/filtersfwd.h"
#include "controllers/direct/filtering/filteringcontrollersfwd.h"
#include "controllers/direct/attributes/kalmanpositionattrcontroller.h"
#include "controllers/switches/singlemodelswitchbase.h"

struct PositionModelSwitch : private SingleModelSwitchBase<PositionFilteringController, KalmanPositionAttrController>
{
    using base_type = SingleModelSwitchBase<PositionFilteringController, KalmanPositionAttrController>;

    PositionModelSwitch(QComboBox * sw, std::shared_ptr<PositionFilteringController> pos_ctrl, std::unique_ptr<KalmanPositionAttrController> attr_ctrl);

    using base_type::enable;
    using base_type::disable;
    using base_type::set_model;
    void switch_model(int cb_idx);

private:
    PositionEKF ekf_filter;
    PositionUKF ukf_filter;
};

#endif // POSITIONMODELSWITCH_H
