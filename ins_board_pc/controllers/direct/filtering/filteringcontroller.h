#ifndef FILTERINGCONTROLLER_H
#define FILTERINGCONTROLLER_H

#include "controllers/direct/filtering/filteringcontrollercommon.h"
#include "controllers/direct/controllerbase.h"
#include "controllers/direct/observablebase.h"
#include "receiver.h"
#include "adapters/positionfilteringviewmodel.h"
#include "adapters/orientationfilteringviewmodel.h"

#include <QPushButton>
#include <QLineEdit>

template<typename Model, typename View>
struct FilteringController : FilteringControllerCommon, ControllerBase<Model>,
                             ObservableBase<View>
{
    FilteringController(const QPushButton * start_button, const QLineEdit * accum_le)
        : FilteringControllerCommon{ start_button->isChecked() }
    {
        connect(start_button, SIGNAL(toggled(bool)), this, SLOT(handle_start(bool)));
        connect(accum_le, SIGNAL(textEdited(QString)), this, SLOT(set_accum_capacity(QString)));
        mvm_adapter.set_accumulator_capacity(accum_le->text().toUInt());
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

private:
    Adapter<Model, typename View::ViewModel> mvm_adapter;
};

#endif // FILTERINGCONTROLLER_H
