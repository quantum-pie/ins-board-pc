#ifndef OBSERVABLEBASE_H
#define OBSERVABLEBASE_H

#include <vector>
#include <functional>
#include <memory>

template<typename View>
struct ObservableBase
{
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
        for(auto & view : views)
        {
            view->update(model);
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
};


#endif // OBSERVABLEBASE_H
