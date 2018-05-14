#ifndef KALMANPOSITIONFILTERBASE_H
#define KALMANPOSITIONFILTERBASE_H

#include "IKalmanPositionFilter.h"
#include "IFilterBase.h"

#include <boost/date_time/gregorian/gregorian.hpp>

class KalmanPositionFilterBase;

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
};

class KalmanPositionFilterBase : virtual public IKalmanPositionFilter,
                                 public IFilterBase<KalmanPositionFilterBase>
{
public:
    explicit KalmanPositionFilterBase(const Ellipsoid & ellip = Ellipsoid::WGS84);
    ~KalmanPositionFilterBase() override;

    using thy_traits = FilterBaseTraits<KalmanPositionFilterBase>;

    using state_type = typename thy_traits::state_type;
    using meas_type = typename thy_traits::meas_type;

    using F_type = typename thy_traits::F_type;
    using P_type = typename thy_traits::P_type;
    using Q_type = typename thy_traits::Q_type;
    using R_type = typename thy_traits::R_type;
    using H_type = typename thy_traits::H_type;

private:
    friend class IFilterBase<KalmanPositionFilterBase>;

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

    /*!
     * @brief Create state transition matrix (F).
     * @param z filter input reference.
     * @return state transition matrix.
     */
    F_type do_create_transition_mtx(const FilterInput & z) const;

    /*!
     * @brief Create initial state estimate covariance matrix (P).
     * @param dt time elapsed since the last step.
     * @return state transition matrix.
     */
    P_type do_create_init_cov_mtx() const;

    /*!
     * @brief Create process noise covariance matrix (Q).
     * @param dt time elapsed since the last step.
     * @return process noise covariance matrix.
     */
    Q_type do_create_proc_noise_cov_mtx(double dt) const;

    /*!
     * @brief Create measurement noise covariance matrix (R).
     * @param geo geodetic coordinates.
     * @param mag_magn magnetic field magnitude.
     * @return measurement noise covariance matrix.
     */
    R_type do_create_meas_noise_cov_mtx(const Vector3D & geo, const boost::gregorian::date & day) const;

    /*!
     * @brief Create state-to-measurement projection matrix (H).
     * @param geo geodetic coordinates.
     * @param earth_model Earth model.
     * @return state-to-measurement projection matrix.
     */
    H_type do_create_meas_proj_mtx(const Vector3D & geo, const boost::gregorian::date & day) const;

    void do_reset() override;

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

    const Ellipsoid ellip;
    state_type x;
    P_type P;

    bool initialized;

    struct
    {
        struct
        {
            double accel_std;       //!< Process noise acceleration standard deviation.
        } proc_params;

        struct
        {
            double gps_cep;         //!< measured position CEP (GPS).
            double gps_vel_std;     //!< measured velocity x std.
        } meas_params;

        struct
        {
            double pos_std;     //!< Initial position estimate standard deviation.
            double vel_std;     //!< Initial velocity estimate standard deviation.
            double accel_std;   //!< Initial acceleration estimate standard deviation.
        } init_params;
    } params;
};

#endif // KALMANPOSITIONFILTERBASE_H
