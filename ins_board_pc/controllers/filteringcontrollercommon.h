#ifndef FILTERINGCONTROLLERCOMMON_H
#define FILTERINGCONTROLLERCOMMON_H

#include <QObject>

class FilterInput;

class FilteringControllerCommon : public QObject
{
    Q_OBJECT

public:
    FilteringControllerCommon(bool already_running)
        : running{ already_running }, filtering_enabled{ true } {}

    bool filtering_is_enabled() { return filtering_enabled; }
    bool is_runnign() { return running; }

    void set_running(bool en) { running = en; }

    void enable_filtering() { filtering_enabled = true; }
    void disable_filtering()
    {
        filtering_enabled = false;
        set_running(false);
    }

public slots:
    void handle_start(bool) {}
    void handle_input(const FilterInput &) {}

private:
    bool running;
    bool filtering_enabled;
};

#endif // FILTERINGCONTROLLERCOMMON_H
