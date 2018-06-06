#ifndef KALMANPOSITIONFILTERBASE_H
#define KALMANPOSITIONFILTERBASE_H

#include "filtering/public_interfaces/IKalmanPositionFilter.h"
#include "filtering/private_implementation/IFilterBase.h"

#include <boost/date_time/gregorian/gregorian.hpp>

class KalmanPositionFilterBase;

/*!
 * @brief Kalman position filter base traits specialization.
 */
template<>
struct FilterBaseTraits<KalmanPositionFilterBase>
{
    static constexpr std::size_t state_size { 9 };
    static constexpr std::size_t measurement_size { 6 };

    using state_type = StaticVector<state_size>;
    using meas_type = StaticVector<measurement_size>;

    using F_type = StaticMatrix<state_size, state_size>;
    using P_type = F_type;
    using Q_type = F_type;
    using R_type = StaticMatrix<measurement_size, measurement_size>;
    using H_type = StaticMatrix<measurement_size, state_size>;
    using K_type = StaticMatrix<state_size, measurement_size>;
};

/*!
 * @brief Kalman position filter base implementation.
 */
struct KalmanPositionFilterBase : virtual IKalmanPositionFilter,
                                  IFilterBase<KalmanPositionFilterBase>
{
    /*!
     * @brief Class constructor.
     * @param ellip Earth ellipsoid.
     */
    explicit KalmanPositionFilterBase(const Ellipsoid & ellip = Ellipsoid::WGS84);

    /*!
     * @brief Class destructor.
     */
    ~KalmanPositionFilterBase() override = default;

    //! Traits of this implementation alias.
    using thy_traits = FilterBaseTraits<KalmanPositionFilterBase>;

    using state_type = typename thy_traits::state_type;
    using meas_type = typename thy_traits::meas_type;

    using F_type = typename thy_traits::F_type;
    using P_type = typename thy_traits::P_type;
    using Q_type = typename thy_traits::Q_type;
    using R_type = typename thy_traits::R_type;
    using H_type = typename thy_traits::H_type;
    using K_type = typename thy_traits::K_type;

protected:
    void do_reset() override;

private:
    friend class IFilterBase<KalmanPositionFilterBase>;

    // Static polymorphism implementation
    meas_type do_get_true_measurement(const FilterInput & z) const;
    meas_type do_get_predicted_measurement(const Vector3D & geo, const boost::gregorian::date & day) const;

    state_type do_get_state() const;
    void do_set_state(const state_type & st);

    P_type do_get_cov() const;
    void do_set_cov(const P_type & P);

    bool do_is_initialized() const;
    bool do_is_ready_to_initialize() const;
    void do_initialize(const FilterInput & z);
    void do_accumulate(const FilterInput & z);

    Vector3D do_get_geodetic(const FilterInput & z) const;

    F_type do_create_transition_mtx(const FilterInput & z) const;
    P_type do_create_init_cov_mtx() const;
    Q_type do_create_proc_noise_cov_mtx(double dt) const;
    R_type do_create_meas_noise_cov_mtx(const Vector3D & geo, const boost::gregorian::date & day) const;
    H_type do_create_meas_proj_mtx(const Vector3D & geo, const boost::gregorian::date & day) const;

    // Dynamic polymorphism implementation
    Ellipsoid do_get_ellipsoid() const override;
    Vector3D do_get_acceleration() const override;
    Vector3D do_get_velocity() const override;
    Vector3D do_get_cartesian() const override;

    void do_set_proc_accel_std(double std) override;
    void do_set_meas_pos_std(double std) override;
    void do_set_meas_vel_std(double std) override;
    void do_set_init_pos_std(double std) override;
    void do_set_init_vel_std(double std) override;
    void do_set_init_accel_std(double std) override;

    double do_get_proc_accel_std() const override;
    double do_get_meas_pos_std() const override;
    double do_get_meas_vel_std() const override;
    double do_get_init_pos_std() const override;
    double do_get_init_vel_std() const override;
    double do_get_init_accel_std() const override;

    const Ellipsoid ellip;      //!< Reference Earth model.
    state_type x;               //!< Filter state.
    P_type P;                   //!< State estimate covariance matrix.

    bool initialized;           //!< Filter is initialized flag.

    struct ProcessNoiseParams
    {
        double accel_std;       //!< Process noise acceleration standard deviation.
    };

    struct MeasurementNoiseParams
    {
        double gps_cep;         //!< measured position CEP (GPS).
        double gps_vel_std;     //!< measured velocity x std.
    };

    struct InitCovParams
    {
        double pos_std;     //!< Initial position estimate standard deviation.
        double vel_std;     //!< Initial velocity estimate standard deviation.
        double accel_std;   //!< Initial acceleration estimate standard deviation.
    };

    struct
    {
        ProcessNoiseParams proc_params;
        MeasurementNoiseParams meas_params;
        InitCovParams init_params;
    } params;

    static const ProcessNoiseParams     default_proc_noise_params;
    static const MeasurementNoiseParams default_meas_noise_params;
    static const InitCovParams          default_init_cov_params;
};

#endif // KALMANPOSITIONFILTERBASE_H
