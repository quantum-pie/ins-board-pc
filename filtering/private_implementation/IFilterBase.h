#ifndef FILTERIMPLINTERFACE_H
#define FILTERIMPLINTERFACE_H

#include "eigenaux.h"

#include <boost/date_time/gregorian/gregorian.hpp>

/*!
 * @brief Filter implementation traits structure.
 * @tparam T filter implementation type.
 */
template<typename T>
struct FilterBaseTraits;

class FilterInput;

/*!
 * @brief Filter implementation CRTP base.
 * @tparam Derived derived implementation.
 */
template<typename Derived>
struct IFilterBase
{
    //! Derived implementation alias.
    using impl_type = Derived;

    //! Traits of derived implementation.
    using derived_traits = FilterBaseTraits<Derived>;

    using state_type =  typename derived_traits::state_type;
    using P_type =      typename derived_traits::P_type;

    /*!
     * @brief Get filter state.
     * @return filter state.
     */
    state_type get_state() const
    {
        return derived().do_get_state();
    }

    /*!
     * @brief Set filter state.
     * @param st filter state reference.
     */
    void set_state(const state_type & st)
    {
        derived().do_set_state(st);
    }

    /*!
     * @brief Get filter state estimate covariance matrix.
     * @return state estimate covariance matrix.
     */
    P_type get_cov() const
    {
        return derived().do_get_cov();
    }

    /*!
     * @brief Set filter state estimate covariance matrix.
     * @param P state estimate covariance matrix reference.
     */
    void set_cov(const P_type & P)
    {
        derived().do_set_cov(P);
    }

    /*!
     * @brief Get measurement vector.
     * @param z filter input.
     * @return measurement vector.
     */
    auto get_true_measurement(const FilterInput & z) const
    {
        return derived().do_get_true_measurement(z);
    }

    /*!
     * @brief Predict measurement.
     * @param geo geodetic coordinates.
     * @param day current date.
     * @return predicted measurement vector.
     */
    auto get_predicted_measurement(const Vector3D & geo, const boost::gregorian::date & day) const
    {
        return derived().do_get_predicted_measurement(geo, day);
    }

    /*!
     * @brief Get geodetic coordinates.
     * @param z filter input.
     * @return geodetic coordinates.
     */
    Vector3D get_geodetic(const FilterInput & z) const
    {
        return derived().do_get_geodetic(z);
    }

    /*!
     * @brief Check if filter is initialized.
     * @return true if is initialized.
     */
    bool is_initialized() const
    {
        return derived().do_is_initialized();
    }

    /*!
     * @brief Check if filter is ready to initialize.
     * @return true of ready.
     */
    bool is_ready_to_initialize() const
    {
        return derived().do_is_ready_to_initialize();
    }

    /*!
     * @brief Initialized filter.
     * @param z filter input.
     */
    void initialize(const FilterInput & z)
    {
        derived().do_initialize(z);
    }

    /*!
     * @brief Accumulate filter input.
     * @param z filter input.
     */
    void accumulate(const FilterInput & z)
    {
        derived().do_accumulate(z);
    }

    /*!
     * @brief Create state transition matrix (F).
     * @param z filter input reference.
     * @return state transition matrix.
     */
    auto create_transition_mtx(const FilterInput & z) const
    {
        return derived().do_create_transition_mtx(z);
    }

    /*!
     * @brief Create initial state estimate covariance matrix (P).
     * @return state transition matrix.
     */
    auto create_init_cov_mtx() const
    {
        return derived().do_create_init_cov_mtx();
    }

    /*!
     * @brief Create process noise covariance matrix (Q).
     * @param dt time elapsed since the last step.
     * @return process noise covariance matrix.
     */
    auto create_proc_noise_cov_mtx(double dt) const
    {
        return derived().do_create_proc_noise_cov_mtx(dt);
    }

    /*!
     * @brief Create measurement noise covariance matrix (R).
     * @param geo geodetic coordinates.
     * @param day current date.
     * @return measurement noise covariance matrix.
     */
    auto create_meas_noise_cov_mtx(const Vector3D & geo, const boost::gregorian::date & day) const
    {
        return derived().do_create_meas_noise_cov_mtx(geo, day);
    }

    /*!
     * @brief Create state-to-measurement projection matrix (H).
     * @param geo geodetic coordinates.
     * @param day current date.
     * @return state-to-measurement projection matrix.
     */
    auto create_meas_proj_mtx(const Vector3D & geo, const boost::gregorian::date & day) const
    {
        return derived().do_create_meas_proj_mtx(geo, day);
    }

private:
    /*!
     * @brief Cast this to derived implementation type.
     * @return derived implementation reference.
     */
    Derived & derived()
    {
        return static_cast<Derived&>(*this);
    }

    /*!
     * @brief Cast this to derived implementation type (const version).
     * @return derived implementation constant reference.
     */
    const Derived & derived() const
    {
        return static_cast<const Derived&>(*this);
    }
};

#endif // FILTERIMPLINTERFACE_H
