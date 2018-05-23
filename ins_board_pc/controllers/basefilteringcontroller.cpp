#include "controllers/filteringcontroller.h"

BaseFilteringController::BaseFilteringController(BaseFilteringModel & model)
    : QObject{ nullptr },
      model { model },
      is_running{ false }
{}

void BaseFilteringController::handle_strategy(IFilter * filter)
{
    model.set_strategy(filter);
    model.reset();
}

void BaseFilteringController::handle_start(bool en)
{
    is_running = en;
    if(en)
    {
        model.reset();
    }
}

void BaseFilteringController::handle_input(const FilterInput & z)
{
    if(is_running)
    {
        model.step(z);
    }
}
