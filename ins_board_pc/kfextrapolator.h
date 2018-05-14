#ifndef KFEXTRAPOLATOR_H
#define KFEXTRAPOLATOR_H

template<typename Base>
class KFExtrapolator : public Base
{
    void extrapolate(const FilterInput & z)
    {
        state_type x = get_state();
        P_type P = get_cov();

        F_type F = create_transition_mtx(z);
        Q_type Q = create_proc_noise_cov_mtx(z);

        set_cov(F * P * F.transpose() + Q);
        set_state(F * x);
    }
}

#endif // KFEXTRAPOLATOR_H
