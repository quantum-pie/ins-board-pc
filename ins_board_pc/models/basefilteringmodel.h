#ifndef IFILTERINGMODEL_H
#define IFILTERINGMODEL_H

#include <QObject>

class FilterInput;

template<typename Filter>
class BaseFilteringModel : QObject
{
    Q_OBJECT

public:
    using filter_type = Filter;

    BaseFilteringModel()
        : QObject{ nullptr } {}

    void set_strategy(Filter * other_filter)
    {
        filtering_strategy = other_filter;
        emit strategy_changed();
    }

    void step(const FilterInput & z)
    {
        filtering_strategy->step(z);
        emit refreshed();
    }

    void reset()
    {
        filtering_strategy->reset();
    }

signals:
    void refreshed() const;
    void strategy_changed() const;

protected:
    Filter * filtering_strategy;
};

#endif // IFILTERINGMODEL_H
