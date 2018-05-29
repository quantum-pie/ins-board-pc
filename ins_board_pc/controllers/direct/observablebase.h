#ifndef OBSERVABLEBASE_H
#define OBSERVABLEBASE_H

#include <vector>

template<typename View>
struct ObservableBase
{
    void attach_view(View & view)
    {
        views.push_back(view);
    }

    void clear_views()
    {
        views.clear();
    }

protected:
    void update_views(const typename View::ViewModel & model)
    {
        for(auto view : views)
        {
            view.get().update(model);
        }
    }

private:
    std::vector<std::reference_wrapper<View>> views;
};


#endif // OBSERVABLEBASE_H
