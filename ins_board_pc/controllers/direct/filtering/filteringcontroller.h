#ifndef FILTERINGCONTROLLER_H
#define FILTERINGCONTROLLER_H

#include "controllers/direct/filtering/filteringcontrollercommon.h"
#include "controllers/controllerbase.h"
#include "controllers/observablebase.h"
#include "communication/receiver.h"
#include "adapters/positionfilteringviewmodel.h"
#include "adapters/orientationfilteringviewmodel.h"

#include <QPushButton>
#include <QLineEdit>

template<typename Model, typename View>
struct FilteringController : FilteringControllerCommon, ControllerBase<Model>,
                             ObservableBase<View>
{
    FilteringController(const QPushButton * start_button, const QPushButton * clear_button, const QLineEdit * accum_le)
        : FilteringControllerCommon{ start_button->isChecked() }
    {
        connect(start_button, SIGNAL(toggled(bool)), this, SLOT(handle_start(bool)));
        connect(clear_button, SIGNAL(released()), this, SLOT(clear_plots()));
        connect(accum_le, SIGNAL(textEdited(QString)), this, SLOT(set_accum_capacity(QString)));
        mvm_adapter.set_accumulator_capacity(accum_le->text().toUInt());
    }

    ~FilteringController() override = default;

    void handle_start(bool en) override
    {
        set_running(en);
        if(en)
        {
            if(filtering_is_enabled() && this->model_is_set())
            {
                this->get_model()->reset();
            }
            this->clear_views();
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
            this->update_views(mvm_adapter(*this->get_model()));
        }
    }

    void set_accum_capacity(const QString & cap) override
    {
        bool ok;
        std::size_t new_capacity = cap.toUInt(&ok);
        if(ok)
        {
            mvm_adapter.set_accumulator_capacity(new_capacity);
        }
    }

    void clear_plots() override
    {
        this->clear_views();
    }

private:
    Adapter<Model, typename View::ViewModel> mvm_adapter;
};

#endif // FILTERINGCONTROLLER_H
