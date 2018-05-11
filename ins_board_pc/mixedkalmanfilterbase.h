#ifndef MIXEDKALMANFILTERBASE_H
#define MIXEDKALMANFILTERBASE_H

#include "IKalmanOrientationFilter.h"
#include "IKalmanPositionFilter.h"
#include "kalmanorientationfilterbase.h"
#include "kalmanpositionfilterbase.h"

#include <boost/date_time/gregorian/gregorian.hpp>

class Earth;

class MixedKalmanFilterBase : KalmanOrientationFilterBase,
                              KalmanPositionFilterBase
{
    explicit MixedKalmanFilterBase(const KalmanOrientationFilterBase::FilterParams & ori_params,
                                   const KalmanPositionFilterBase::FilterParams & pos_params);

    ~MixedKalmanFilterBase() override;

    static constexpr std::size_t ori_state_size { KalmanOrientationFilterBase::state_size };
    static constexpr std::size_t pos_state_size { KalmanPositionFilterBase::state_size };
    static constexpr std::size_t ori_meas_size { KalmanOrientationFilterBase::measurement_size };
    static constexpr std::size_t pos_meas_size { KalmanPositionFilterBase::measurement_size };

    static constexpr std::size_t state_size { ori_state_size + pos_state_size };
    static constexpr std::size_t measurement_size { ori_meas_size + pos_meas_size };

    using F_type = StaticMatrix<state_size, state_size>;
    using P_type = F_type;
    using Q_type = F_type;
    using R_type = StaticMatrix<measurement_size, measurement_size>;
    using H_type = StaticMatrix<measurement_size, state_size>;

    /*!
     * @brief Create state transition matrix (F).
     * @param z filter input reference.
     * @return state transition matrix.
     */
    F_type create_transition_mtx(const FilterInput & z) const;

    /*!
     * @brief Create initial state estimate covariance matrix (P).
     * @param dt time elapsed since the last step.
     * @return state transition matrix.
     */
    P_type create_init_cov_mtx() const;

    /*!
     * @brief Create process noise covariance matrix (Q).
     * @param dt time elapsed since the last step.
     * @return process noise covariance matrix.
     */
    Q_type create_proc_noise_cov_mtx(double dt) const;

    /*!
     * @brief Create measurement noise covariance matrix (R).
     * @param geo geodetic coordinates.
     * @param mag_magn magnetic field magnitude.
     * @return measurement noise covariance matrix.
     */
    R_type create_meas_noise_cov_mtx(const Vector3D & geo, double mag_magn) const;

    /*!
     * @brief Create state-to-measurement projection matrix (H).
     * @param geo geodetic coordinates.
     * @param earth_model Earth model.
     * @return state-to-measurement projection matrix.
     */
    H_type create_meas_proj_mtx(const Vector3D & geo, const boost::gregorian::date & day,
                                const Earth & earth_model) const;
};

#endif // MIXEDKALMANFILTERBASE_H
