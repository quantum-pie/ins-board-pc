#ifndef FILTERINGCONTROLLER_H
#define FILTERINGCONTROLLER_H

#include "controllers/direct/filtering/filteringcontrollercommon.h"
#include "controllers/direct/controllerbase.h"
#include "controllers/direct/observablebase.h"
#include "receiver.h"

#include <QPushButton>

template<typename Model, typename View>
struct FilteringController : ControllerBase<Model>, ObservableBase<View>,
                             private FilteringControllerCommon
{
    FilteringController(const QPushButton * start_button)
        : FilteringControllerCommon{ start_button->isChecked() }
    {
        connect(start_button, SIGNAL(toggled(bool)), this, SLOT(handle_start(bool)));
    }

    using FilteringControllerCommon::is_running;
    using FilteringControllerCommon::set_running;
    using FilteringControllerCommon::filtering_is_enabled;
    using FilteringControllerCommon::enable_filtering;
    using FilteringControllerCommon::disable_filtering;

    void handle_start(bool en)
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

    void handle_input(const FilterInput & z)
    {
        if(this->model_is_set())
        {
            if(is_running() && filtering_is_enabled())
            {
                this->get_model()->step(z);
            }
            this->update_views(typename View::ViewModel{*this->get_model(), z});
        }
    }
};

#endif // FILTERINGCONTROLLER_H
