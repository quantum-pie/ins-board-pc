#ifndef UKFCORRECTOR_H
#define UKFCORRECTOR_H

#include "eigenaux.h"
#include "packets.h"

#include "filtering/private_implementation/IFilterBase.h"
#include "filtering/plugins/filterplugins.h"

#include <cmath>
#include <array>

#include <Eigen/Dense>

template<typename Base>
struct UKFCorrector : ICorrector<UKFCorrector<Base>>, Base
{
    static_assert(std::is_base_of<IFilterBase<typename Base::impl_type>, Base>::value,
                  "Base class do not inherit IFilterBase CRTP");

    UKFCorrector() : params{ default_ut_params }
    {
        double alpha_sq = params.alpha * params.alpha;
        lambda = alpha_sq * (L + params.kappa) - L;

        Ws[0] = lambda / (L + lambda);
        Wc[0] = Ws[0] + (1 - alpha_sq + params.beta);

        for(int i = 1; i <= 2 * L; ++i)
        {
            Ws[i] = 0.5 / (L + lambda);
            Wc[i] = 0.5 / (L + lambda);
        }
    }

    void do_correct(const FilterInput & z)
    {
        auto x = this->get_state();
        auto P = this->get_cov();

        sigma_p[0] = x;

        P_type mtx_root = P.llt().matrixL();
        mtx_root *= std::sqrt(L + lambda);

        for(int i = 1; i <= L; ++i)
        {
            sigma_p[i] = x + mtx_root.col(i - 1);
            sigma_p[i + L] = x - mtx_root.col(i - 1);
        }

        meas_type z_p = meas_type::Zero();
        for(int i = 0; i <= 2 * L; ++i)
        {
            this->set_state(sigma_p[i]);
            Vector3D geo = this->get_geodetic(z);

            sigma_z[i] = this->get_predicted_measurement(geo, z.day);
            z_p += Ws[i] * sigma_z[i];
        }

        R_type Pzz = R_type::Zero();
        K_type Pxz = K_type::Zero();

        for(int i = 0; i <= 2 * L; ++i)
        {
            Pzz += Wc[i] * (sigma_z[i] - z_p) * (sigma_z[i] - z_p).transpose();
            Pxz += Wc[i] * (sigma_p[i] - sigma_p[0]) * (sigma_z[i] - z_p).transpose();
        }

        this->set_state(sigma_p[0]);
        Vector3D geo = this->get_geodetic(z);

        auto R = this->create_meas_noise_cov_mtx(geo, z.day);
        Pzz += R;

        auto K = Pxz * Pzz.inverse();
        this->set_cov(P - K * Pzz * K.transpose());

        auto z_meas = this->get_true_measurement(z);
        this->set_state(sigma_p[0] + K * (z_meas - z_p));
    }

private:
    using state_type = typename Base::state_type;
    using meas_type = typename Base::meas_type;
    using P_type = typename Base::P_type;
    using R_type = typename Base::R_type;
    using K_type = typename Base::K_type;

    static constexpr int L { Base::thy_traits::state_size };    //!< Augmented state size.
    double lambda;                                              //!< Unscented transform lambda parameter.

    std::array<double, 2 * L + 1> Ws;
    std::array<double, 2 * L + 1> Wc;

    std::array<state_type, 2 * L + 1> sigma_p;
    std::array<meas_type, 2 * L + 1> sigma_z;

    /*!
     * @brief Parameters of unscented transform.
     */
    struct UnscentedTransformParams
    {
        double kappa;       //!< Kappa.
        double beta;        //!< Beta.
        double alpha;       //!< Alpha.
    } params;

    static constexpr UnscentedTransformParams default_ut_params{ 0, 2, 1e-3 };
};

#endif // UKFCORRECTOR_H
