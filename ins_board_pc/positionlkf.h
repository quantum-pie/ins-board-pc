/*
 * positionlkf.h
 *
 *      Author: Ermakov_P
 */

#ifndef INCLUDE_POSITIONLKF_H_
#define INCLUDE_POSITIONLKF_H_

#include "IKalmanPositionFilter.h"
#include "earth.h"

/*!
 * @brief Concrete Kalman linear position filter.
 */
class PositionLKF final : public IKalmanPositionFilter
{
public:
    /*!
     * @brief Kalman filter parameters structure.
     */
    struct FilterParams
    {
        ProcessNoiseParams proc_params;         //!< Process noise parameters instance.
        MeasurementNoiseParams meas_params;     //!< Measurement noise parameters instance.
        InitCovParams init_params;              //!< Initial state estimate covariance parameters instance.
    };

    /*!
     * @brief Class constructor.
     * @param params filter parameters.
     */
    explicit PositionLKF(const FilterParams & params);

    /*!
     * @brief Class destructor.
     */
    ~PositionLKF() override;

    /* IKalmanPositionFilter interface implementation */
    void step(const FilterInput & z) override;
    void reset() override;

	Vector3D get_cartesian() const override;
    Ellipsoid get_ellipsoid() const override;
	Vector3D get_velocity() const override;
	Vector3D get_acceleration() const override;

    void set_proc_accel_std(double std) override;
    void set_meas_pos_std(double std) override;
    void set_meas_vel_std(double std) override;
    void set_init_pos_std(double std) override;
    void set_init_vel_std(double std) override;
    void set_init_accel_std(double std) override;

    double get_proc_accel_std() const override;
    double get_meas_pos_std() const override;
    double get_meas_vel_std() const override;
    double get_init_pos_std() const override;
    double get_init_vel_std() const override;
    double get_init_accel_std() const override;

private:
    /*!
     * @brief Step of initialized filter.
     * @param z filter input.
     */
    void step_initialized(const FilterInput & z);

    /*!
     * @brief Step of uninitialized filter.
     * @param z filter input.
     */
    void step_uninitialized(const FilterInput & z);

    static constexpr int state_size { 9 };        	//!< Size of state vector.
    static constexpr int measurement_size { 6 };  	//!< Size of measurements vector.
    static constexpr int process_noise_size { 3 };  //!< Size of process noise vector.

    /* Useful aliases */
    using state_type = StaticVector<state_size>;
    using meas_type = StaticVector<measurement_size>;

    using F_type = StaticMatrix<state_size, state_size>;
    using Q_type = F_type;
    using P_type = F_type;
    using R_type = StaticMatrix<measurement_size, measurement_size>;
    using H_type = StaticMatrix<measurement_size, state_size>;
    using S_type = R_type;
    using K_type = StaticMatrix<state_size, measurement_size>;
    using G_type = StaticMatrix<state_size, process_noise_size>;

    /*!
     * @brief Create state transition matrix (F).
     * @param dt time elapsed since the last step.
     * @return state transition matrix.
     */
    F_type create_transition_mtx(double dt) const;

    /*!
     * @brief Create process noise covariance matrix (Q).
     * @param dt time elapsed since the last step.
     * @return process noise covariance matrix.
     */
    Q_type create_proc_noise_cov_mtx(double dt) const;

    /*!
     * @brief Create measurement noise covariance matrix (R).
     * @param geo geodetic coordinates vector.
     * @return measurement noise covariance matrix.
     */
    R_type create_meas_noise_cov_mtx(const Vector3D & geo) const;

    /*!
     * @brief Create ENU system measurement covariance matrix.
     * @return local ENU measurement noise covariance matrix.
     */
    Matrix3D create_local_cov_mtx() const;

    /*!
     * @brief Create state-to-measurement projection matrix (H).
     * @return state-to-measurement projection matrix.
     */
    H_type create_meas_proj_mtx() const;

    bool is_initialized;                //!< Filter is initialized flag.

    P_type P;                        	//!< State estimate covariance matrix.
    Matrix3D local_cov;                 //!< Local ENU measurement noise covariance matrix.

    FilterParams params;                //!< Filter parameters instance.
    state_type x;                       //!< State vector.
};

#endif /* INCLUDE_POSITIONLKF_H_ */
