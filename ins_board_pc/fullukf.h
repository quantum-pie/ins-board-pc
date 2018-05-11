/*! \file fullukf.h
  */

#ifndef FULLUKF_H
#define FULLUKF_H

#include "IKalmanPositionFilter.h"
#include "IKalmanOrientationFilter.h"

#include "mixedkalmanfilterbase.h"
#include "kfextrapolator.h"
#include "ukfcorrector.h"

#include "qualitycontrol.h"

// TODO what to do with initialization (inherit from base?)

/*!
 * @brief Concrete Unscented Kalman filter for simultaneous orientation and position estimation.
 */
class FullUKF final : virtual public IKalmanOrientationFilter,
                      virtual public IKalmanPositionFilter,
                      KFExtrapolator<MixedKalmanFilterBase>,
                      UKFCorrector<MixedKalmanFilterBase>
{
public:
    using OrientationFilterParams = MixedKalmanFilterBase::KalmanOrientationFilterBase::FilterParams;
    using PositionFilterParams = MixedKalmanFilterBase::KalmanPositionFilterBase::FilterParams;
    using UTParams = UKFCorrector::UnscentedTransformParams;

    /*!
     * @brief Constructor.
     * @param params filter parameters.
     */
    FullUKF(const OrientationFilterParams & ori_params,
            const PositionFilterParams & pos_params,
            const UTParams & ut_params,
            const Ellipsoid & ellip = Ellipsoid::WGS84);

    /*!
     * \brief Destructor.
     */
    ~FullUKF() override;

private:
    /* Interfaces implementation */
    void do_step(const FilterInput & z) override;
    void do_reset() override;

    bool is_initialized;                    //!< Filter is initialized flag.
    QualityControl<Vector3D> bias_ctrl;     //!< Gyroscope bias controller.

    P_type P;                               //!< State estimate covariance matrix.
};


#endif // FULLUKF_H
