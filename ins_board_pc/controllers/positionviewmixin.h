#ifndef POSITIONVIEWMIXIN_H
#define POSITIONVIEWMIXIN_H

#include <vector>
#include <functional>

class FilterInput;
class IPositionView;

template<typename BaseController>
struct PositionViewMixin : BaseController
{
public:
    void handle_input(const FilterInput & z)
    {
        BaseController::handle_input(z);
    }

private:
    std::vector<std::reference_wrapper<IPositionView>> views;
}

#endif // POSITIONVIEWMIXIN_H
