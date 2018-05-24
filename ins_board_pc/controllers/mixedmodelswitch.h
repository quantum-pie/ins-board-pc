#ifndef MIXEDMODELSWITCH_H
#define MIXEDMODELSWITCH_H

#include "controllers/modelswitchbase.h"
#include "filtering/filters/generickalman.h"
#include "controllers/positionmodelswitch.h"
#include "controllers/orientationmodelswitch.h"

#include <QComboBox>

class MixedModelSwitch : public ModelSwitchBase
{
public:
    MixedModelSwitch(QComboBox * sw,
                     PositionModelSwitch & pos_sw,
                     OrientationModelSwitch & ori_sw)
        : ModelSwitchBase{ sw },
          pos_sw{ pos_sw }, ori_sw{ ori_sw }
    {
        connect(sw, SIGNAL(currentIndexChanged(int)), this, SLOT(switch_model(int)));
    }

    void switch_model(int cb_idx)
    {
        if(cb_idx == 0)
        {
            pos_sw.set_model(&ekf_filter);
            ori_sw.set_model(&ekf_filter);
        }
        else if(cb_idx == 1)
        {
            pos_sw.set_model(&ukf_filter);
            ori_sw.set_model(&ukf_filter);
        }
    }

private:
    PositionModelSwitch & pos_sw;
    OrientationModelSwitch & ori_sw;

    FullEKF ekf_filter;
    FullUKF ukf_filter;
};

#endif // MIXEDMODELSWITCH_H
