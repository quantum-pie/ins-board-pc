#ifndef REMOTECONTROLLER_H
#define REMOTECONTROLLER_H

#include "controllers/observablebase.h"
#include "controllers/remote/filtering/remotebase.h"
#include "adapters/orientationfilteringviewmodel.h"
#include "adapters/positionfilteringviewmodel.h"

#include <QPushButton>
#include <QLineEdit>

template<typename View>
struct RemoteController : RemoteBase, ObservableBase<View>
{
    RemoteController(QPushButton * clear_btn, QLineEdit * accum_le)
    {
        connect(clear_btn, SIGNAL(released()), this, SLOT(clear_plots()));
        connect(accum_le, SIGNAL(textEdited(QString)), this, SLOT(set_accum_capacity(QString)));
        mvm_adapter.set_accumulator_capacity(accum_le->text().toUInt());
    }

    void handle_input(const FilteredPacket & z) override
    {
        this->update_views(mvm_adapter(z));
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
    Adapter<FilteredPacket, typename View::ViewModel> mvm_adapter;
};

#endif // REMOTECONTROLLER_H
