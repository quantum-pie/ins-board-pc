#include "controllers/direct/filtering/filteringcontrollercommon.h"

FilteringControllerCommon::FilteringControllerCommon(bool already_running)
    : running{ already_running }, filtering_enabled{ true } {}

bool FilteringControllerCommon::filtering_is_enabled()
{
    return filtering_enabled;
}

bool FilteringControllerCommon::is_running()
{
    return running;
}

void FilteringControllerCommon::set_running(bool en)
{
    running = en;
}

void FilteringControllerCommon::enable_filtering()
{
    filtering_enabled = true;
}

void FilteringControllerCommon::disable_filtering()
{
    filtering_enabled = false;
    set_running(false);
}
