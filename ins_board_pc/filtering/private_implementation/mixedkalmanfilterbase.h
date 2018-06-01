#ifndef MIXEDKALMANFILTERBASE_H
#define MIXEDKALMANFILTERBASE_H

#include "filtering/public_interfaces/IKalmanOrientationFilter.h"
#include "filtering/public_interfaces/IKalmanPositionFilter.h"

#include "filtering/private_implementation/IFilterBase.h"
#include "filtering/private_implementation/kalmanorientationfilterbase.h"
#include "filtering/private_implementation/kalmanpositionfilterbase.h"

#include <boost/date_time/gregorian/gregorian.hpp>

class MixedKalmanFilterBase;

/*!
 * @brief Kalman mixed filter base traits specialization.
 */
template<>
struct FilterBaseTraits<MixedKalmanFilterBase>
{
    using OrientationBaseTraits = FilterBaseTraits<KalmanOrientationFilterBase>;
    using PositionBaseTraits = FilterBaseTraits<KalmanPositionFilterBase>;

    static constexpr std::size_t state_size { OrientationBaseTraits::state_size + PositionBaseTraits::state_size };
    static constexpr std::size_t measurement_size { OrientationBaseTraits::measurement_size + PositionBaseTraits::measurement_size };

    using state_type = StaticVector<state_size>;
    using meas_type = StaticVector<measurement_size>;

    using F_type = StaticMatrix<state_size, state_size>;
    using P_type = F_type;
    using Q_type = F_type;
    using R_type = StaticMatrix<measurement_size, measurement_size>;
    using H_type = StaticMatrix<measurement_size, state_size>;
    using K_type = StaticMatrix<state_size, measurement_size>;
    using PLL_type = StaticMatrix<PositionBaseTraits::state_size, OrientationBaseTraits::state_size>;
    using PUR_type = StaticMatrix<OrientationBaseTraits::state_size, PositionBaseTraits::state_size>;
};

/*!
 * @brief Kalman mixed filter base implementation.
 */
class MixedKalmanFilterBase : public IFilterBase<MixedKalmanFilterBase>,
                              KalmanOrientationFilterBase,
                              KalmanPositionFilterBase
{
public:
    /*!
     * @brief Class constructor.
     * @param ellip Earth ellipsoid.
     */
    explicit MixedKalmanFilterBase(const Ellipsoid & ellip = Ellipsoid::WGS84);

    /*!
     * @brief Class destructor.
     */
    ~MixedKalmanFilterBase() override = default;

    //! Traits of this implementation alias.
    using thy_traits = FilterBaseTraits<MixedKalmanFilterBase>;

    static constexpr std::size_t ori_state_size { thy_traits::OrientationBaseTraits::state_size };
    static constexpr std::size_t pos_state_size { thy_traits::PositionBaseTraits::state_size };
    static constexpr std::size_t ori_meas_size { thy_traits::OrientationBaseTraits::measurement_size };
    static constexpr std::size_t pos_meas_size { thy_traits::PositionBaseTraits::measurement_size };

    using state_type = typename thy_traits::state_type;
    using meas_type = typename thy_traits::meas_type;

    using F_type = typename thy_traits::F_type;
    using P_type = typename thy_traits::P_type;
    using Q_type = typename thy_traits::Q_type;
    using R_type = typename thy_traits::R_type;
    using H_type = typename thy_traits::H_type;
    using K_type = typename thy_traits::K_type;
    using PLL_type = typename thy_traits::PLL_type;
    using PUR_type = typename thy_traits::PUR_type;

    // Specify which CRTP base interface to use
    using CRTPBase = IFilterBase<MixedKalmanFilterBase>;
    using impl_type = CRTPBase::impl_type;
    using CRTPBase::accumulate;
    using CRTPBase::create_init_cov_mtx;
    using CRTPBase::create_meas_noise_cov_mtx;
    using CRTPBase::create_meas_proj_mtx;
    using CRTPBase::create_proc_noise_cov_mtx;
    using CRTPBase::create_transition_mtx;
    using CRTPBase::get_cov;
    using CRTPBase::get_geodetic;
    using CRTPBase::get_predicted_measurement;
    using CRTPBase::get_state;
    using CRTPBase::get_true_measurement;
    using CRTPBase::initialize;
    using CRTPBase::is_initialized;
    using CRTPBase::is_ready_to_initialize;
    using CRTPBase::set_cov;
    using CRTPBase::set_state;

private:
    friend class IFilterBase<MixedKalmanFilterBase>;

    // Static polymorphism implementation
    meas_type do_get_true_measurement(const FilterInput & z) const;
    meas_type do_get_predicted_measurement(const Vector3D & geo, const boost::gregorian::date & day) const;

    state_type do_get_state() const;
    void do_set_state(const state_type & st);

    P_type do_get_cov() const;
    void do_set_cov(const P_type & P);

    Vector3D do_get_geodetic(const FilterInput & z) const;

    bool do_is_initialized() const;
    bool do_is_ready_to_initialize() const;
    void do_initialize(const FilterInput & z);
    void do_accumulate(const FilterInput & z);

    F_type do_create_transition_mtx(const FilterInput & z) const;
    P_type do_create_init_cov_mtx() const;
    Q_type do_create_proc_noise_cov_mtx(double dt) const;
    R_type do_create_meas_noise_cov_mtx(const Vector3D & geo, const boost::gregorian::date & day) const;
    H_type do_create_meas_proj_mtx(const Vector3D & geo, const boost::gregorian::date & day) const;

    // Dynamic polymorphism implementation
    void do_reset() override;

    PLL_type PLL;   //!< Lower left state estimate covariance matrix block.
    PUR_type PUR;   //!< Upper right state estimate covariance matrix block.
};

#endif // MIXEDKALMANFILTERBASE_H
