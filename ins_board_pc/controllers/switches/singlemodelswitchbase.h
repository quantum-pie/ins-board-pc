#ifndef SINGLEMODELSWITCH_H
#define SINGLEMODELSWITCH_H

#include "controllers/switches/modelswitchbase.h"
#include "controllers/direct/controllerbase.h"

template<typename FilteringController, typename AttributesController>
struct SingleModelSwitchBase : ModelSwitchBase
{
    SingleModelSwitchBase(QComboBox * sw, FilteringController & fctrl, AttributesController & actrl)
        : ModelSwitchBase{ sw }, fctrl{ fctrl }, actrl{ actrl }
    {}

    using ModelSwitchBase::enable;
    using ModelSwitchBase::disable;

    template<typename Filter>
    void set_model(Filter * new_model)
    {
        new_model->reset();
        fctrl.set_model(new_model);
        actrl.set_model(new_model);
        actrl.apply_attributes();
    }

private:
    FilteringController & fctrl;
    AttributesController & actrl;
};

#endif // SINGLEMODELSWITCH_H
