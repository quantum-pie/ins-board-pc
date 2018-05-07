/*! \file fullukf.h
  */

#ifndef FULLUKF_H
#define FULLUKF_H

#include "IKalmanPositionFilter.h"
#include "IKalmanOrientationFilter.h"
#include "qualitycontrol.h"
#include "earth.h"

#include <array>

/*!
 * @brief Concrete Unscented Kalman filter for simultaneous orientation and position estimation.
 */
class FullUKF final : public IKalmanOrientationFilter, public IKalmanPositionFilter
{
public:
    /*!
     * @brief Parameters of unscented transform.
     */
    struct UnscentedTransformParams
    {
        double kappa;       //!< Kappa.
        double beta;        //!< Beta.
        double alpha;       //!< Alpha.
    };

    /*!
     * @brief Kalman filter parameters structure.
     */
    struct FilterParams
    {
        IKalmanPositionFilter::ProcessNoiseParams pos_proc_params;          //!< Position part process noise parameters.
        IKalmanOrientationFilter::ProcessNoiseParams ori_proc_params;       //!< Orientation part process noise parameters.
        IKalmanPositionFilter::MeasurementNoiseParams pos_meas_params;      //!< Position part measurement noise parameters.
        IKalmanOrientationFilter::MeasurementNoiseParams ori_meas_params;   //!< Orientation part measurement noise parameters.
        IKalmanPositionFilter::InitCovParams pos_init_params;               //!< Position part initial state estimate covariance parameters.
        IKalmanOrientationFilter::InitCovParams ori_init_params;            //!< Orientation part initial state estimate covariance parameters.
        const UnscentedTransformParams ut_params;                           //!< Unscented transform parameters instance.
        std::size_t accum_capacity;                                         //!< Capacity of filter input accumulator.
    };

    /*!
     * @brief Constructor.
     * @param params filter parameters.
     */
    explicit FullUKF(const FilterParams & params);

    /*!
     * \brief Destructor.
     */
    ~FullUKF() override;

    /* Interfaces implementation */
    void step(const FilterInput & z) override;
    void reset() override;

    Vector3D get_cartesian() const override;
    Ellipsoid get_ellipsoid() const override;
    Vector3D get_velocity() const override;
    Vector3D get_acceleration() const override;

    quat::Quaternion get_orientation_quaternion() const override;
    Vector3D get_gyro_bias() const override;

    void set_proc_accel_std(double std) override;
    void set_meas_pos_std(double std) override;
    void set_meas_vel_std(double std) override;
    void set_init_pos_std(double std) override;
    void set_init_vel_std(double std) override;
    void set_init_accel_std(double std) override;

    void set_proc_gyro_std(double std) override;
    void set_proc_gyro_bias_std(double std) override;
    void set_meas_accel_std(double std) override;
    void set_meas_magn_std(double std) override;
    void set_init_qs_std(double std) override;
    void set_init_qx_std(double std) override;
    void set_init_qy_std(double std) override;
    void set_init_qz_std(double std) override;
    void set_init_bias_std(double std) override;

    double get_proc_accel_std() const override;
    double get_meas_pos_std() const override;
    double get_meas_vel_std() const override;
    double get_init_pos_std() const override;
    double get_init_vel_std() const override;
    double get_init_accel_std() const override;

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

    static constexpr int state_size { 16 };        	//!< Size of state vector.
    static constexpr int measurement_size { 12 };  	//!< Size of measurements vector.

    /* Useful aliases */
    using state_type = StaticVector<state_size>;
    using meas_type = StaticVector<measurement_size>;

    using F_type = StaticMatrix<state_size, state_size>;
    using Q_type = F_type;
    using P_type = F_type;
    using R_type = StaticMatrix<measurement_size, measurement_size>;
    using K_type = StaticMatrix<state_size, measurement_size>;

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
     * @brief Create ENU system measurement covariance matrix.
     * @return local ENU measurement noise covariance matrix.
     */
    Matrix3D create_local_cov_mtx() const;

    bool is_initialized;                    //!< Filter is initialized flag.

    P_type P;                               //!< State estimate covariance matrix.
    Matrix3D local_cov;                     //!< Local ENU measurement noise covariance matrix.

    FilterParams params;                    //!< Filter parameters instance.
    QualityControl<Vector3D> bias_ctrl;     //!< Gyroscope bias controller.
    state_type x;                           //!< State vector.

    const Earth earth_model;                //!< Reference Earth model.

    static constexpr int L {state_size};    //!< Augmented state size.
    double lambda;                          //!< Unscented transform lambda parameter.

    std::array<double, 2 * L + 1> Ws;
    std::array<double, 2 * L + 1> Wc;

    std::array<state_type, 2 * L + 1> sigma_p;
    std::array<meas_type, 2 * L + 1> sigma_z;
};


#endif // FULLUKF_H
