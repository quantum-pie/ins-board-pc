#ifndef UKFCORRECTOR_H
#define UKFCORRECTOR_H

#include "eigenaux.h"

#include <cmath>
#include <array>

#include <Eigen/Dense>

template<typename Base>
class UKFCorrector : virtual Base
{
    /*!
     * @brief Parameters of unscented transform.
     */
    struct UnscentedTransformParams
    {
        double kappa;       //!< Kappa.
        double beta;        //!< Beta.
        double alpha;       //!< Alpha.
    };

    EKFCorrector(const UnscentedTransformParams & ut_params)
        : params{ ut_params }
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

    void correct(const FilterInput & z, P_type & P)
    {
        state_type x = get_state();

        if(z.gps_valid)
        {
            // update (unscented version)
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
                set_state(sigma_p[i]);
                Vector3D geo = get_geodetic(z);

                sigma_z[i] = predicted_measurements(geo, z.day);
                z_p += Ws[i] * sigma_z[i];
            }

            R_type Pzz = R_type::Zero();
            K_type Pxz = K_type::Zero();

            for(int i = 0; i <= 2 * L; ++i)
            {
                Pzz += Wc[i] * (sigma_z[i] - z_p) * (sigma_z[i] - z_p).transpose();
                Pxz += Wc[i] * (sigma_p[i] - sigma_p[0] ) * (sigma_z[i] - z_p).transpose();
            }

            set_state(sigma_p[0]);
            Vector3D geo = get_geodetic(z);

            R_type R = create_meas_noise_cov_mtx(geo, z.day);
            Pzz += R;

            K_type K = Pxz * Pzz.inverse();
            P -= K * Pzz * K.transpose();

            meas_type z_meas = true_measurements(z);
            set_state(sigma_p[0] + K * (z_meas - z_p));
        }
    }

private:
    static constexpr int L { state_size };    //!< Augmented state size.
    double lambda;                            //!< Unscented transform lambda parameter.

    UnscentedTransformParams params;

    std::array<double, 2 * L + 1> Ws;
    std::array<double, 2 * L + 1> Wc;

    std::array<state_type, 2 * L + 1> sigma_p;
    std::array<meas_type, 2 * L + 1> sigma_z;
}

#endif // UKFCORRECTOR_H
