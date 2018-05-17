#ifndef IFILTERINGCONTROLLER_H
#define IFILTERINGCONTROLLER_H

// Refactor to templated base using templated model as member

#include <QObject>

class FilterInput;

struct IFilteringController : QObject
{
    virtual ~IFilteringController() {}

private slots:
    virtual void handle_start(bool en) = 0;
    virtual void handle_strategy(int idx) = 0;
    virtual void handle_input(const FilterInput & z) = 0;

protected:
    IFilteringController(QObject *parent = nullptr)
        : QObject(parent) {}

private:
    Q_OBJECT
};

#endif // IFILTERINGCONTROLLER_H
