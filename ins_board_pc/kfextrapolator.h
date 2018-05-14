#ifndef KFEXTRAPOLATOR_H
#define KFEXTRAPOLATOR_H

#include "IFilterBase.h"
#include "filterplugins.h"

template<typename FilterBase>
class KFExtrapolator : public IExtrapolator<KFExtrapolator<FilterBase>>,
                       FilterBase
{
    std::enable_if_t<std::is_base_of<IFilterBase<FilterBase>, FilterBase>::value>
    do_extrapolate(const FilterInput & z)
    {
        typename FilterBase::state_type x = FilterBase::get_state();
        typename FilterBase::P_type P = FilterBase::get_cov();

        typename FilterBase::F_type F = FilterBase::create_transition_mtx(z);
        typename FilterBase::Q_type Q = FilterBase::create_proc_noise_cov_mtx(z);

        FilterBase::set_cov(F * P * F.transpose() + Q);
        FilterBase::set_state(F * x);
    }

    void do_step(const FilterInput & z) override
    {

    }
};

#endif // KFEXTRAPOLATOR_H
