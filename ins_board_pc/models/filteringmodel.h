#ifndef FILTERINGMODEL_H
#define FILTERINGMODEL_H

#include "filtering/public_interfaces/IFilter.h"

#include <QObject>

class FilteringModel : public QObject
{
public:
    explicit FilteringModel(IFilter * wrapee)
        : QObject{ nullptr }, wrapee{ wrapee } {}

    ~FilteringModel() override = default;

    void step(const FilterInput & z)
    {
        wrapee->step(z);
        emit model_updated();
    }

    void reset()
    {
        wrapee->reset();
    }

signals:
    void model_updated();

private:
    Q_OBJECT

    IFilter * wrapee;
};

#endif // FILTERINGMODEL_H
