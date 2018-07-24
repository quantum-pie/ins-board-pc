#ifndef GENERICKALMAN_H
#define GENERICKALMAN_H

#include "filtering/plugins/filterplugins.h"

#include <type_traits>

/*!
 * @brief Generic Kalman filter class.
 *
 * This class implements Kalman filter interface privately inheriting from specific implementation.
 * @tparam Implementation implementation class.
 * @tparam Interfaces interfaces to implement.
 */
template<typename Implementation, typename... Interfaces>
class GenericKalmanFilter final : public virtual Interfaces..., Implementation
{
    // Ensure that provided implementation has IExtrapolator and ICorrector plugins.
    static_assert(std::is_base_of<IExtrapolator<typename Implementation::exptrapolator_base>, Implementation>::value &&
                  std::is_base_of<ICorrector<typename Implementation::corrector_base>, Implementation>::value,
                  "Extrapolator and Corrector interfaces are not implemented");

    void do_step(const FilterInput & z) override
    {
        if(this->is_initialized())
        {
            this->extrapolate(z);

            if(z.gps_valid)
                this->correct(z);
        }
        else if(this->is_ready_to_initialize())
        {
            this->initialize(z);
        }
        else
        {
            this->accumulate(z);
        }
    }
};

#endif // GENERICKALMAN_H
