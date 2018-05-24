#ifndef IFILTERINGCONTROLLER_H
#define IFILTERINGCONTROLLER_H

#include "controllers/filteringcontrollerbase.h"
#include "receiver.h"
#include <QPushButton>

template<typename Model, typename View>
class FilteringController : public FilteringControllerBase
{
public:
    FilteringController(const QPushButton * start_button,
                        const Receiver * receiver)
        : model{ nullptr }, is_running{ start_button->isChecked() }, is_enabled{ false }
    {
        connect(start_button, SIGNAL(toggled(bool)), this, SLOT(handle_start(bool)));
        connect(receiver, SIGNAL(raw_sample_received(FilterInput)), this, SLOT(handle_input(FilterInput)));
    }

    void set_model(Model * new_model)
    {
        model = new_model;
    }

    void handle_start(bool en) override
    {
        if(is_enabled)
        {
            is_running = en;
            if(en && model)
            {
                model->reset();
            }
        }
    }

    void handle_input(const FilterInput & z) override
    {
        if(model)
        {
            if(is_running)
            {
                model->step(z);
            }

        }
    }

private:
    Model * model;
    bool is_running;
    bool is_enabled;
};

#endif // IFILTERINGCONTROLLER_H
