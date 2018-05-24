#ifndef FILTERINGCONTROLLER_H
#define FILTERINGCONTROLLER_H

#include "controllers/filteringcontrollercommon.h"
#include "controllers/controllerbase.h"
#include "receiver.h"
#include <QPushButton>

#include "views/IBaseView.h"
#include "filtering/public_interfaces/IOrientationFilter.h"
#include "filtering/public_interfaces/IPositionFilter.h"

template<typename Model, typename View>
class FilteringController : public FilteringControllerCommon, public ControllerBase<Model>
{
public:
    FilteringController(const QPushButton * start_button,
                        const Receiver * receiver)
        : FilteringControllerCommon{ start_button->isChecked() }
    {
        connect(start_button, SIGNAL(toggled(bool)), this, SLOT(handle_start(bool)));
        connect(receiver, SIGNAL(raw_sample_received(FilterInput)), this, SLOT(handle_input(FilterInput)));
    }

    void attach_view(View & view)
    {
        views.push_back(view);
    }

    void clear_views()
    {
        views.clear();
    }

    void handle_start(bool en)
    {
        if(filtering_is_enabled())
        {
            set_running(en);
            if(en && this->model_is_set())
            {
                this->get_model()->reset();
            }
        }
    }

    void handle_input(const FilterInput & z)
    {
        if(this->model_is_set())
        {
            if(is_runnign() && filtering_is_enabled())
            {
                this->get_model()->step(z);
            }
            update_views(this->get_model());
        }
    }

private:
    void update_views(Model * model)
    {
        for(auto view : views)
        {
            view.get().update(model);
        }
    }

    std::vector<std::reference_wrapper<View>> views;
};

using PositionFilteringController = FilteringController<IPositionFilter, IPositionView>;
using OrientationFilteringController = FilteringController<IOrientationFilter, IOrientationView>;

#endif // FILTERINGCONTROLLER_H
