#ifndef EKFCORRECTOR_H
#define EKFCORRECTOR_H

#include "filtering/private_implementation/IFilterBase.h"
#include "filtering/plugins/filterplugins.h"

#include "packets.h"

/*!
 * @brief Extended Kalman filter correction prccedure mixin.
 * @tparam Base base filter implementation.
 */
template<typename Base>
struct EKFCorrector : ICorrector<EKFCorrector<Base>>, Base
{
    // Ensure that provided Base implements IFilterBase CRTP interface.
    static_assert(std::is_base_of<IFilterBase<typename Base::impl_type>, Base>::value,
                  "Base class do not inherit IFilterBase CRTP");

    void do_correct(const FilterInput & z)
    {
        auto x = this->get_state();
        auto P = this->get_cov();

        Vector3D geo = this->get_geodetic(z);

        auto z_meas = this->get_true_measurement(z);
        auto z_pr = this->get_predicted_measurement(geo, z.day);

        auto R = this->create_meas_noise_cov_mtx(geo, z.day);
        auto H = this->create_meas_proj_mtx(geo, z.day);

        auto S = H * P * H.transpose() + R;
        auto K = P * H.transpose() * S.inverse();

        this->set_state(x + K * (z_meas - z_pr));

        auto tmp = Base::P_type::Identity() - K * H;
        this->set_cov(tmp * P * tmp.transpose() + K * R * K.transpose());
    }
};

#endif // EKFCORRECTOR_H
