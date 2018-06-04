#ifndef SINGLEMODELSWITCH_H
#define SINGLEMODELSWITCH_H

#include "controllers/switches/modelswitchbase.h"
#include "controllers/direct/controllerbase.h"

#include <memory>

template<typename FilteringController, typename AttributesController>
struct SingleModelSwitchBase : ModelSwitchBase
{
    SingleModelSwitchBase(QComboBox * sw, std::shared_ptr<FilteringController> fctrl, std::unique_ptr<AttributesController> actrl)
        : ModelSwitchBase{ sw }, fctrl{ fctrl }, actrl{ std::move(actrl) }
    {}

    using ModelSwitchBase::enable;
    using ModelSwitchBase::disable;

    template<typename Filter>
    void set_model(Filter * new_model)
    {
        new_model->reset();
        fctrl->set_model(new_model);
        actrl->set_model(new_model);
        actrl->apply_attributes();
    }

private:
    std::shared_ptr<FilteringController> fctrl;
    std::unique_ptr<AttributesController> actrl;
};

#endif // SINGLEMODELSWITCH_H
