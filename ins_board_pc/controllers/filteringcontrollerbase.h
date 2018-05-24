#ifndef FILTERINGCONTROLLERBASE_H
#define FILTERINGCONTROLLERBASE_H

#include <QObject>

class FilterInput;

class FilteringControllerBase : public QObject
{
    Q_OBJECT

public slots:
    virtual void handle_start(bool en) = 0;
    virtual void handle_input(const FilterInput & z) = 0;
};

#endif // FILTERINGCONTROLLERBASE_H
