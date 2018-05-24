#ifndef ORIENTATIONMODELSWITCH_H
#define ORIENTATIONMODELSWITCH_H

#include "filtering/public_interfaces/IOrientationFilter.h"
#include "filtering/filters/generickalman.h"
#include "controllers/singlemodelswitchbase.h"

class OrientationModelSwitch : public SingleModelSwitchBase<IOrientationFilter>
{
public:
    OrientationModelSwitch(QComboBox * sw) : SingleModelSwitchBase<IOrientationFilter>(sw)
    {
        connect(sw, SIGNAL(currentIndexChanged(int)), this, SLOT(switch_model(int)));
    }

    void switch_model(int cb_idx)
    {
        if(cb_idx == 0)
        {
            set_model(&ekf_filter);
        }
        else if(cb_idx == 1)
        {
            set_model(&ukf_filter);
        }
    }

private:
    OrientationEKF ekf_filter;
    OrientationUKF ukf_filter;
};

#endif // ORIENTATIONMODELSWITCH_H
