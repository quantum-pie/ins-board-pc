#include "controllers/direct/filtering/filteringcontrollercommon.h"

FilteringControllerCommon::FilteringControllerCommon(bool already_running)
    : RunningFlag{ already_running }, filtering_enabled{ true } {}

bool FilteringControllerCommon::filtering_is_enabled()
{
    return filtering_enabled;
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
