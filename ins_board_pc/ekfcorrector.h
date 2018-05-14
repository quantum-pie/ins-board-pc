#ifndef EKFCORRECTOR_H
#define EKFCORRECTOR_H

#include "eigenaux.h"

template<typename Base>
class EKFCorrector : public Base
{
    void correct(const FilterInput & z)
    {
        state_type x = get_state();
        P_type P = get_cov();

        if(z.gps_valid)
        {
            Vector3D geo = get_geodetic(z);

            meas_type z_meas = true_measurements(z);
            meas_type z_pr = predicted_measurements(geo, z.day);

            auto y = z_meas - z_pr;

            R_type R = create_meas_noise_cov_mtx(geo, z.day);
            H_type H = create_meas_proj_mtx(geo, z.day);

            auto S = H * P * H.transpose() + R;
            auto K = P * H.transpose() * S.inverse();

            set_state(x + K * y);

            auto tmp = P_type::Identity() - K * H;
            set_cov(tmp * P * tmp.transpose() + K * R * K.transpose());
        }
    }
}

#endif // EKFCORRECTOR_H
