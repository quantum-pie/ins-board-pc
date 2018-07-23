#ifndef OBSERVABLEBASE_H
#define OBSERVABLEBASE_H

#include <vector>
#include <functional>
#include <memory>

/*!
 * @brief Base class for anything that can have views.
 * @tparam View view type.
 */
template<typename View>
struct ObservableBase
{
    /*!
     * @brief ObservableBase constructor.
     * @param decimation rate of views update decimation.
     */
    explicit ObservableBase(std::size_t decimation = 2)
        : decimation{ decimation }, counter { 0 }
    {}

    /*!
     * @brief Attach new view.
     * @param view new view.
     */
    void attach_view(std::shared_ptr<View> view)
    {
        views.emplace_back(view);
    }

    /*!
     * @brief Remove all views.
     */
    void remove_views()
    {
        views.clear();
    }

protected:
    /*!
     * @brief Update views.
     * @param model ViewModel reference.
     */
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

    /*!
     * @brief Clear all views.
     */
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
