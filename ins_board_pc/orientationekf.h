/*! \file orientationekf.h
  */

#ifndef ORIENTATIONEKF_H
#define ORIENTATIONEKF_H

#include "IKalmanOrientationFilter.h"
#include "qualitycontrol.h"
#include "earth.h"

/*!
 * @brief Concrete Kalman filter for orientation estimation.
 */
class OrientationEKF final : public IKalmanOrientationFilter
{
public:
    /*!
     * @brief Kalman filter parameters structure.
     */
    struct FilterParams
    {
        ProcessNoiseParams proc_params;     //!< Process noise parameters instance.
        MeasurementNoiseParams meas_params; //!< Measurement noise parameters instance.
        InitCovParams init_params;          //!< Initial state estimate covariance parameters instance.
        std::size_t accum_capacity;         //!< Capacity of filter input accumulator.
    };

    /*!
     * @brief Constructor.
     * @param params filter parameters.
     */
    explicit OrientationEKF(const FilterParams & params);

    /*!
     * \brief Destructor.
     */
    ~OrientationEKF() override;

    /* Interfaces implementation */
    void step(const FilterInput & z) override;
    void reset() override;

    quat::Quaternion get_orientation_quaternion() const override;
    Vector3D get_gyro_bias() const override;

    void set_proc_gyro_std(double std) override;
    void set_proc_gyro_bias_std(double std) override;
    void set_meas_accel_std(double std) override;
    void set_meas_magn_std(double std) override;
    void set_init_qs_std(double std) override;
    void set_init_qx_std(double std) override;
    void set_init_qy_std(double std) override;
    void set_init_qz_std(double std) override;
    void set_init_bias_std(double std) override;

    double get_proc_gyro_std() const override;
    double get_proc_gyro_bias_std() const override;
    double get_meas_accel_std() const override;
    double get_meas_magn_std() const override;
    double get_init_qs_std() const override;
    double get_init_qx_std() const override;
    double get_init_qy_std() const override;
    double get_init_qz_std() const override;
    double get_init_bias_std() const override;

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

    /*!
     * @brief Initialize filter.
     * @param z filter input.
     */
    void initialize(const FilterInput & z);

    /*!
     * @brief normalize filter state.
     */
    void normalize_state();

    static constexpr int state_size { 7 };        	//!< Size of state vector.
    static constexpr int measurement_size { 6 };  	//!< Size of measurements vector.

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
    using V_type = quat::skew_type;
    using D_type = quat::delta_type;

    /*!
     * @brief Create state transition matrix (F).
     * @param z filter input reference.
     * @return state transition matrix.
     */
    F_type create_transition_mtx(const FilterInput & z) const;

    /*!
     * @brief Create process noise covariance matrix (Q).
     * @param dt time elapsed since the last step.
     * @return process noise covariance matrix.
     */
    Q_type create_proc_noise_cov_mtx(double dt) const;

    /*!
     * @brief Create measurement noise covariance matrix (R).
     * @param z filter input reference.
     * @return measurement noise covariance matrix.
     */
    R_type create_meas_noise_cov_mtx(const Vector3D & geo,
                                     const boost::gregorian::date & day) const;

    /*!
     * @brief Create state-to-measurement projection matrix (H).
     * @param z filter input reference.
     * @return state-to-measurement projection matrix.
     */
    H_type create_meas_proj_mtx(const Vector3D & geo,
                                const boost::gregorian::date & day) const;

    bool is_initialized;                //!< Filter is initialized flag.

    P_type P;                           //!< State estimate covariance matrix.

    FilterParams params;                //!< Filter parameters instance.
    QualityControl<Vector3D> bias_ctrl; //!< Gyroscope bias controller.
    state_type x;                       //!< State vector.

    const Earth earth_model;            //!< Reference Earth model.
};

#endif // ORIENTATIONEKF_H
