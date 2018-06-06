#ifndef FILTERINGCONTROLLERCOMMON_H
#define FILTERINGCONTROLLERCOMMON_H

#include "controllers/direct/runningflag.h"

#include <QObject>

#include <QDebug>

class FilterInput;

class FilteringControllerCommon : public QObject, RunningFlag
{
    Q_OBJECT

public:
    FilteringControllerCommon(bool already_running);
    virtual ~FilteringControllerCommon() = default;

    using RunningFlag::is_running;
    using RunningFlag::set_running;

    bool filtering_is_enabled();
    void enable_filtering();
    void disable_filtering();

public slots:
    virtual void handle_start(bool)
    {
        qDebug() << "Handle start of filtering controller\n";
    }

    virtual void handle_input(const FilterInput &)
    {
        qDebug() << "Handle input of filtering controller\n";
    }

private:
    bool filtering_enabled;
};

#endif // FILTERINGCONTROLLERCOMMON_H
