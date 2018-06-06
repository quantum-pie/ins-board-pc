#ifndef FILTERINGCONTROLLERCOMMON_H
#define FILTERINGCONTROLLERCOMMON_H

#include "controllers/direct/runningflag.h"

#include <QObject>

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
    virtual void handle_start(bool) = 0;
    virtual void handle_input(const FilterInput &) = 0;

private:
    bool filtering_enabled;
};

#endif // FILTERINGCONTROLLERCOMMON_H
