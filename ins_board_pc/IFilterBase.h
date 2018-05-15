#ifndef FILTERIMPLINTERFACE_H
#define FILTERIMPLINTERFACE_H

#include "eigenaux.h"

#include <boost/date_time/gregorian/gregorian.hpp>

template<typename T>
struct FilterBaseTraits;

class FilterInput;

template<typename Derived>
struct IFilterBase
{
    using impl_type = Derived;
    using derived_traits = FilterBaseTraits<Derived>;

    using state_type =  typename derived_traits::state_type;
    using P_type =      typename derived_traits::P_type;

    state_type get_state() const
    {
        return derived().do_get_state();
    }

    void set_state(const state_type & st)
    {
        derived().do_set_state(st);
    }

    P_type get_cov() const
    {
        return derived().do_get_cov();
    }

    void set_cov(const P_type & P)
    {
        derived().do_set_cov(P);
    }

    auto get_true_measurement(const FilterInput & z) const
    {
        return derived().do_get_true_measurement(z);
    }

    auto get_predicted_measurement(const Vector3D & geo, const boost::gregorian::date & day) const
    {
        return derived().do_get_predicted_measurement(geo, day);
    }

    Vector3D get_geodetic(const FilterInput & z) const
    {
        return derived().do_get_geodetic(z);
    }

    bool is_initialized() const
    {
        return derived().do_is_initialized();
    }

    bool is_ready_to_initialize() const
    {
        return derived().do_is_ready_to_initialize();
    }

    void initialize(const FilterInput & z)
    {
        derived().do_initialize(z);
    }

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
     * @param dt time elapsed since the last step.
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
     * @param mag_magn magnetic field magnitude.
     * @return measurement noise covariance matrix.
     */
    auto create_meas_noise_cov_mtx(const Vector3D & geo, const boost::gregorian::date & day) const
    {
        return derived().do_create_meas_noise_cov_mtx(geo, day);
    }

    /*!
     * @brief Create state-to-measurement projection matrix (H).
     * @param geo geodetic coordinates.
     * @param earth_model Earth model.
     * @return state-to-measurement projection matrix.
     */
    auto create_meas_proj_mtx(const Vector3D & geo, const boost::gregorian::date & day) const
    {
        return derived().do_create_meas_proj_mtx(geo, day);
    }

private:
    Derived & derived()
    {
        return static_cast<Derived&>(*this);
    }

    const Derived & derived() const
    {
        return static_cast<const Derived&>(*this);
    }
};

#endif // FILTERIMPLINTERFACE_H
