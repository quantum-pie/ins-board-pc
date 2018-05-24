#ifndef SINGLEMODELSWITCH_H
#define SINGLEMODELSWITCH_H

#include "controllers/modelswitchbase.h"
#include "controllers/controllerbase.h"

#include <vector>
#include <functional>

class QComboBox;

template<typename Model>
class SingleModelSwitchBase : public ModelSwitchBase
{
public:
    using base_model_controller = ControllerBase<Model>;

    SingleModelSwitchBase(QComboBox * sw)
        : ModelSwitchBase{ sw }
    {}

    void attach_controller(base_model_controller & ctrl)
    {
        model_controllers.push_back(ctrl);
    }

    void clear_controllers()
    {
        model_controllers.clear();
    }

    void set_model(Model * new_model)
    {
        for(auto ctrl : model_controllers)
        {
            ctrl.get().set_model(new_model);
        }
    }

private:
    std::vector<std::reference_wrapper<base_model_controller>> model_controllers;
};

#endif // SINGLEMODELSWITCH_H
