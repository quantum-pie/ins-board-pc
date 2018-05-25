#ifndef FILTERINGCONTROLLERCOMMON_H
#define FILTERINGCONTROLLERCOMMON_H

#include <QObject>

class FilterInput;

class FilteringControllerCommon : public QObject
{
    Q_OBJECT

public:
    FilteringControllerCommon(bool already_running);

    bool is_running();
    void set_running(bool en);

    bool filtering_is_enabled();
    void enable_filtering();
    void disable_filtering();

public slots:
    void handle_start(bool) {}
    void handle_input(const FilterInput &) {}

private:
    bool running;
    bool filtering_enabled;
};

#endif // FILTERINGCONTROLLERCOMMON_H
