#ifndef IFILTERINGCONTROLLER_H
#define IFILTERINGCONTROLLER_H

#include "models/basefilteringmodel.h"

#include <QObject>

class FilterInput;

template<typename Model>
class BaseFilteringController : public QObject
{
    Q_OBJECT

public:
    BaseFilteringController(Model & model)
        : QObject{ nullptr },
          model { model },
          is_running{ false }
    {}

    void handle_strategy(typename Model::filter_type * filter)
    {
        model.set_strategy(filter);
        model.reset();
    }

public slots:
    void handle_start(bool en)
    {
        is_running = en;
        if(en)
        {
            model.reset();
        }
    }

    void handle_input(const FilterInput & z)
    {
        if(is_running)
        {
            model.step(z);
        }
    }

protected:
    Model & model;

private:
    bool is_running;
};

#endif // IFILTERINGCONTROLLER_H
