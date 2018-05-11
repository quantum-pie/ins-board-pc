#ifndef KFEXTRAPOLATOR_H
#define KFEXTRAPOLATOR_H

template<typename Base>
class KFExtrapolator : virtual Base
{
    void extrapolate(const FilterInput & z, P_type & P)
    {
        state_type x = get_state();

        F_type F = create_transition_mtx(z);
        Q_type Q = create_proc_noise_cov_mtx(z);

        P = F * P * F.transpose() + Q;
        set_state(F * x);
    }
}

#endif // KFEXTRAPOLATOR_H
