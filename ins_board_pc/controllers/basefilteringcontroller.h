#ifndef IFILTERINGCONTROLLER_H
#define IFILTERINGCONTROLLER_H

#include "models/filteringmodelbase.h"

#include <QObject>

class FilterInput;

class BaseFilteringController : public QObject
{
    Q_OBJECT

public:
    explicit BaseFilteringController(BaseFilteringModel & model);
    void handle_strategy(IFilter * filter);

public slots:
    void handle_start(bool en);
    void handle_input(const FilterInput & z);

private:
    BaseFilteringModel & model;
    bool is_running;
};

#endif // IFILTERINGCONTROLLER_H
