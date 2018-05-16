#ifndef GENERICKALMAN_H
#define GENERICKALMAN_H

#include "filtering/public_interfaces/IKalmanOrientationFilter.h"
#include "filtering/public_interfaces/IKalmanPositionFilter.h"

#include "filtering/private_implementation/kalmanorientationfilterbase.h"
#include "filtering/private_implementation/kalmanpositionfilterbase.h"
#include "filtering/private_implementation/mixedkalmanfilterbase.h"

#include "filtering/plugins/kfextrapolator.h"
#include "filtering/plugins/ekfcorrector.h"
#include "filtering/plugins/ukfcorrector.h"

/*!
 * @brief Generic Kalman filter class.
 *
 * This class implements Kalman filter interface privately inheriting from specific implementation.
 * @tparam Implementation implementation class.
 * @tparam Interfaces interfaces to implement.
 */
template<typename Implementation, typename... Interfaces>
class GenericKalmanFilter : public virtual Interfaces..., Implementation
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

// Typedefs for concrete Kalman filters.
using PositionEKF =     GenericKalmanFilter<EKFCorrector<KFExtrapolator<KalmanPositionFilterBase>>, IKalmanPositionFilter>;
using PositionUKF =     GenericKalmanFilter<UKFCorrector<KFExtrapolator<KalmanPositionFilterBase>>, IKalmanPositionFilter>;
using OrientationEKF =  GenericKalmanFilter<EKFCorrector<KFExtrapolator<KalmanOrientationFilterBase>>, IKalmanOrientationFilter>;
using OrientationUKF =  GenericKalmanFilter<UKFCorrector<KFExtrapolator<KalmanOrientationFilterBase>>, IKalmanOrientationFilter>;
using FullEKF =         GenericKalmanFilter<EKFCorrector<KFExtrapolator<MixedKalmanFilterBase>>, IKalmanOrientationFilter, IKalmanPositionFilter>;
using FullUKF =         GenericKalmanFilter<UKFCorrector<KFExtrapolator<MixedKalmanFilterBase>>, IKalmanOrientationFilter, IKalmanPositionFilter>;

#endif // GENERICKALMAN_H
