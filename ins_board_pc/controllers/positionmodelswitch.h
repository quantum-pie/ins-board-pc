#ifndef POSITIONMODELSWITCH_H
#define POSITIONMODELSWITCH_H

#include "filtering/public_interfaces/IPositionFilter.h"
#include "filtering/filters/generickalman.h"
#include "controllers/singlemodelswitchbase.h"

class PositionModelSwitch : public SingleModelSwitchBase<IPositionFilter>
{
public:
    PositionModelSwitch(QComboBox * sw) : SingleModelSwitchBase<IPositionFilter>(sw)
    {
        connect(sw, SIGNAL(currentIndexChanged(int)), this, SLOT(switch_model(int)));
    }

    void switch_model(int cb_idx)
    {
        if(cb_idx == 0)
        {
            this->set_model(&ekf_filter);
        }
        else if(cb_idx == 1)
        {
            this->set_model(&ukf_filter);
        }
    }

private:
    PositionEKF ekf_filter;
    PositionUKF ukf_filter;
};

#endif // POSITIONMODELSWITCH_H
