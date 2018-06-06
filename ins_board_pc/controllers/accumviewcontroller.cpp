#include "controllers/accumviewcontroller.h"

#include <QLineEdit>

const std::size_t AccumViewController::default_capacity { 200 };

AccumViewController::AccumViewController(QLineEdit *samples_le)
{
    samples_le->setText(QString::number(default_capacity));
    connect(samples_le, SIGNAL(textEdited(QString)), this, SLOT(accum_capacity_changed(QString)));
}

void AccumViewController::attach_view(std::shared_ptr<IAccumView> view)
{
    views.emplace_back(view);
}

void AccumViewController::remove_views()
{
    views.clear();
}

void AccumViewController::accum_capacity_changed(const QString &str)
{
    for(auto & view : views)
    {
        bool conv_ok;
        std::size_t capacity = str.toUInt(&conv_ok);
        if(conv_ok)
        {
            view->set_accumulator_capacity(capacity);
        }
    }
}
