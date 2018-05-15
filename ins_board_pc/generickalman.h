#ifndef GENERICKALMAN_H
#define GENERICKALMAN_H

#include "IKalmanOrientationFilter.h"
#include "IKalmanPositionFilter.h"
#include "kalmanorientationfilterbase.h"
#include "kalmanpositionfilterbase.h"
#include "mixedkalmanfilterbase.h"
#include "kfextrapolator.h"
#include "ekfcorrector.h"
#include "ukfcorrector.h"

struct IMixedKalmanFilter : virtual IKalmanOrientationFilter, virtual IKalmanPositionFilter {};

template<typename Interface, typename Implementation>
class GenericKalmanFilter : public virtual Interface,
                            Implementation
{
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

using PositionEKF =     GenericKalmanFilter<IKalmanPositionFilter, EKFCorrector<KFExtrapolator<KalmanPositionFilterBase>>>;
using PositionUKF =     GenericKalmanFilter<IKalmanPositionFilter, UKFCorrector<KFExtrapolator<KalmanPositionFilterBase>>>;
using OrientationEKF =  GenericKalmanFilter<IKalmanOrientationFilter, EKFCorrector<KFExtrapolator<KalmanOrientationFilterBase>>>;
using OrientationUKF =  GenericKalmanFilter<IKalmanOrientationFilter, UKFCorrector<KFExtrapolator<KalmanOrientationFilterBase>>>;
using FullEKF =         GenericKalmanFilter<IMixedKalmanFilter, EKFCorrector<KFExtrapolator<MixedKalmanFilterBase>>>;
using FullUKF =         GenericKalmanFilter<IMixedKalmanFilter, UKFCorrector<KFExtrapolator<MixedKalmanFilterBase>>>;

#endif // GENERICKALMAN_H
