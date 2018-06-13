#ifndef FILTERINGCONTROLLER_H
#define FILTERINGCONTROLLER_H

#include "controllers/direct/filtering/filteringcontrollercommon.h"
#include "controllers/direct/controllerbase.h"
#include "controllers/direct/observablebase.h"
#include "receiver.h"
#include "views/IBaseView.h"

#include <QPushButton>

template<typename Model, typename View>
struct FilteringController : FilteringControllerCommon, ControllerBase<Model>,
                             ObservableBase<View>
{
    FilteringController(const QPushButton * start_button)
        : FilteringControllerCommon{ start_button->isChecked() }
    {
        connect(start_button, SIGNAL(toggled(bool)), this, SLOT(handle_start(bool)));
    }

    ~FilteringController() override = default;

    void handle_start(bool en) override
    {
        if(filtering_is_enabled())
        {
            set_running(en);
            if(en && this->model_is_set())
            {
                this->get_model()->reset();
                this->clear_views();
            }
        }
    }

    void handle_input(const FilterInput & z) override
    {
        if(this->model_is_set() && is_running())
        {
            if(filtering_is_enabled())
            {
                this->get_model()->step(z);
            }
            this->update_views(*this->get_model());
        }
    }
};

#endif // FILTERINGCONTROLLER_H
