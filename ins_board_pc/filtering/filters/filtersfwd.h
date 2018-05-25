#ifndef FILTERSFWD_H
#define FILTERSFWD_H

#include "filtering/public_interfaces/IKalmanOrientationFilter.h"
#include "filtering/public_interfaces/IKalmanPositionFilter.h"

#include "filtering/private_implementation/kalmanorientationfilterbase.h"
#include "filtering/private_implementation/kalmanpositionfilterbase.h"
#include "filtering/private_implementation/mixedkalmanfilterbase.h"

#include "filtering/plugins/kfextrapolator.h"
#include "filtering/plugins/ekfcorrector.h"
#include "filtering/plugins/ukfcorrector.h"

#include "filtering/filters/generickalman.h"

// Typedefs for concrete Kalman filters.
using PositionEKF =     GenericKalmanFilter<EKFCorrector<KFExtrapolator<KalmanPositionFilterBase>>, IKalmanPositionFilter>;
using PositionUKF =     GenericKalmanFilter<UKFCorrector<KFExtrapolator<KalmanPositionFilterBase>>, IKalmanPositionFilter>;
using OrientationEKF =  GenericKalmanFilter<EKFCorrector<KFExtrapolator<KalmanOrientationFilterBase>>, IKalmanOrientationFilter>;
using OrientationUKF =  GenericKalmanFilter<UKFCorrector<KFExtrapolator<KalmanOrientationFilterBase>>, IKalmanOrientationFilter>;
using FullEKF =         GenericKalmanFilter<EKFCorrector<KFExtrapolator<MixedKalmanFilterBase>>, IKalmanOrientationFilter, IKalmanPositionFilter>;
using FullUKF =         GenericKalmanFilter<UKFCorrector<KFExtrapolator<MixedKalmanFilterBase>>, IKalmanOrientationFilter, IKalmanPositionFilter>;

#endif // FILTERSFWD_H
