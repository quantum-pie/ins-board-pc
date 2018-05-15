#ifndef KFEXTRAPOLATOR_H
#define KFEXTRAPOLATOR_H

#include "IFilterBase.h"
#include "filterplugins.h"
#include "packets.h"

template<typename Base>
struct KFExtrapolator : IExtrapolator<KFExtrapolator<Base>>,
                        Base
{
    static_assert(std::is_base_of<IFilterBase<typename Base::impl_type>, Base>::value, "Base class do not inherit IFilterBase CRTP");

    void do_extrapolate(const FilterInput & z)
    {
        auto x = this->get_state();
        auto P = this->get_cov();

        auto F = this->create_transition_mtx(z);
        auto Q = this->create_proc_noise_cov_mtx(z.dt);

        this->set_cov(F * P * F.transpose() + Q);
        this->set_state(F * x);
    }
};

#endif // KFEXTRAPOLATOR_H
