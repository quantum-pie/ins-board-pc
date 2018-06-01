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

    using RunningFlag::is_running;
    using RunningFlag::set_running;

    bool filtering_is_enabled();
    void enable_filtering();
    void disable_filtering();

public slots:
    void handle_start(bool) {}
    void handle_input(const FilterInput &) {}

private:
    bool filtering_enabled;
};

#endif // FILTERINGCONTROLLERCOMMON_H
