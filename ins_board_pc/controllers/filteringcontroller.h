#ifndef IFILTERINGCONTROLLER_H
#define IFILTERINGCONTROLLER_H

#include "models/filteringmodel.h"

#include <QObject>

#include <memory>

class FilterInput;

class FilteringController : public QObject
{
    Q_OBJECT

public:
    using model_ptr = std::shared_ptr<FilteringModel>;

    explicit FilteringController(model_ptr model);
    void handle_strategy(IFilter * filter);

public slots:
    void handle_start(bool en);
    void handle_input(const FilterInput & z);

signals:
    void model_switched();

private:
    model_ptr model;
    bool is_running;
};

#endif // IFILTERINGCONTROLLER_H
