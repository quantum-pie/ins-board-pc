#ifndef IFILTERINGMODEL_H
#define IFILTERINGMODEL_H

#include <QObject>
#include <memory>

class FilterInput;

template<typename Filter>
class BaseFilteringModel : QObject
{
    Q_OBJECT

public:
    BaseFilteringModel(QObject *parent = nullptr)
        : QObject(parent) {}

    void set_strategy(std::shared_ptr<Filter> other_filter)
    {
        filtering_strategy = other_filter;
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

protected:
    std::shared_ptr<Filter> filtering_strategy;
};

#endif // IFILTERINGMODEL_H
