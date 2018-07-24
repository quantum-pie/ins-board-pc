#ifndef SINGLEMODELSWITCH_H
#define SINGLEMODELSWITCH_H

#include "controllers/switches/modelswitchbase.h"
#include "controllers/controllerbase.h"

#include <memory>

/*!
 * @brief This is the generic filtering model switch base.
 * @tparam FilteringController filtering controller type.
 * @tparam AttributesController filter attributes controller type.
 */
template<typename FilteringController, typename AttributesController>
struct SingleModelSwitchBase : ModelSwitchBase
{
    /*!
     * @brief SingleModelSwitchBase constructor.
     * @param sw Switch widget.
     * @param fctrl Filtering controller.
     * @param actrl Attributes controller.
     */
    SingleModelSwitchBase(QComboBox * sw, std::shared_ptr<FilteringController> fctrl, std::unique_ptr<AttributesController> actrl)
        : ModelSwitchBase{ sw }, fctrl{ fctrl }, actrl{ std::move(actrl) }
    {}

    ~SingleModelSwitchBase() override = default;

    /*!
     * @brief Set filtering model.
     * @tparam Filter Model type.
     * @param new_model new model pointer.
     */
    template<typename Filter>
    void set_model(Filter * new_model)
    {
        new_model->reset();
        fctrl->set_model(new_model);
        actrl->set_model(new_model);
        actrl->borrow_attributes();
    }

private:
    std::shared_ptr<FilteringController> fctrl;
    std::unique_ptr<AttributesController> actrl;
};

#endif // SINGLEMODELSWITCH_H
