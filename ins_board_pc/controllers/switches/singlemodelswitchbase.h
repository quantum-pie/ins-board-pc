#ifndef SINGLEMODELSWITCH_H
#define SINGLEMODELSWITCH_H

#include "controllers/switches/modelswitchbase.h"
#include "controllers/direct/controllerbase.h"

template<typename FilteringModel, typename AttrModel>
struct SingleModelSwitchBase : ModelSwitchBase
{
    using fmodel_controller = ControllerBase<FilteringModel>;
    using amodel_controller = ControllerBase<AttrModel>;

    SingleModelSwitchBase(QComboBox * sw, fmodel_controller & fctrl, amodel_controller & actrl)
        : ModelSwitchBase{ sw }, fctrl{ fctrl }, actrl{ actrl }
    {}

    template<typename Filter>
    void set_model(Filter * new_model)
    {
        fctrl.set_model(new_model);
        actrl.set_model(new_model);
    }

private:
    fmodel_controller & fctrl;
    amodel_controller & actrl;
};

#endif // SINGLEMODELSWITCH_H