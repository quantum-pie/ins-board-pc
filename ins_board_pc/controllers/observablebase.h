#ifndef OBSERVABLEBASE_H
#define OBSERVABLEBASE_H

#include <vector>
#include <functional>
#include <memory>

template<typename View>
struct ObservableBase
{
    explicit ObservableBase(std::size_t decimation = 20)
        : decimation{ decimation }, counter { 0 }
    {}

    void attach_view(std::shared_ptr<View> view)
    {
        views.emplace_back(view);
    }

    void remove_views()
    {
        views.clear();
    }

protected:
    void update_views(const typename View::ViewModel & model)
    {
        if(++counter == decimation)
        {
            for(auto & view : views)
            {
                view->update(model);
            }
            counter = 0;
        }
    }

    void clear_views()
    {
        for(auto & view : views)
        {
            view->clear();
        }
    }

private:
    std::vector<std::shared_ptr<View>> views;
    const std::size_t decimation;
    std::size_t counter;
};


#endif // OBSERVABLEBASE_H
