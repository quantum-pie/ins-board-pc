#ifndef KALMANORIENTATIONFILTERBASE_H
#define KALMANORIENTATIONFILTERBASE_H

#include "IKalmanOrientationFilter.h"
#include "quatfwd.h"
#include "qualitycontrol.h"
#include "earth.h"

#include <boost/date_time/gregorian/gregorian.hpp>

class KalmanOrientationFilterBase : virtual public IKalmanOrientationFilter
{
public:
    /*!
     * @brief Kalman filter process noise parameters.
     */
    struct ProcessNoiseParams
    {
        double gyro_std;        //!< Process noise gyroscope standard deviation.
        double gyro_bias_std;   //!< Process noise gyroscope bias standard deviation.
    };

    /*!
     * @brief Kalman filter measurement noise parameters.
     */
    struct MeasurementNoiseParams
    {
        double accel_std;           //!< accelerometer measurements std.
        double magn_std;            //!< magnetometer measurements std.
    };

    /*!
     * @brief Kalman filter initial state estimate covariance parameters.
     */
    struct InitCovParams
    {
        double qs_std;      //!< Initial qs estimate standard deviation.
        double qx_std;      //!< Initial qx estimate standard deviation.
        double qy_std;      //!< Initial qy estimate standard deviation.
        double qz_std;      //!< Initial qz estimate standard deviation.
        double bias_std;    //!< Initial gyro bias estimate standard deviation.
    };

    /*!
     * @brief Kalman filter parameters structure.
     */
    struct FilterParams
    {
        ProcessNoiseParams proc_params;     //!< Process noise parameters instance.
        MeasurementNoiseParams meas_params; //!< Measurement noise parameters instance.
        InitCovParams init_params;          //!< Initial state estimate covariance parameters instance.
    };

    explicit KalmanOrientationFilterBase(const FilterParams & params, const Ellipsoid & ellip = Ellipsoid::WGS84);
    ~KalmanOrientationFilterBase() override;

    static constexpr std::size_t state_size { 7 };
    static constexpr std::size_t measurement_size { 6 };

    using state_type = StaticVector<state_size>;
    using meas_type = StaticVector<measurement_size>;

    using F_type = StaticMatrix<state_size, state_size>;
    using P_type = F_type;
    using Q_type = F_type;
    using R_type = StaticMatrix<measurement_size, measurement_size>;
    using H_type = StaticMatrix<measurement_size, state_size>;
    using V_type = quat::skew_type;
    using D_type = quat::delta_type;

    meas_type true_measurement(const FilterInput & z) const;
    meas_type predicted_measurement(const Vector3D & geo, const boost::gregorian::date & day) const;

    state_type get_state() const;
    void set_state(const state_type & st);

    P_type get_cov() const;
    void set_cov(const P_type & P);

    Vector3D get_geodetic(const FilterInput & z) const;

    bool is_initialized() const;
    bool is_ready_to_initialize() const;
    void initialize(const FilterInput & z);
    void accumulate(const FilterInput & z);

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
    R_type create_meas_noise_cov_mtx(const Vector3D & geo, const boost::gregorian::date & day) const;

    /*!
     * @brief Create state-to-measurement projection matrix (H).
     * @param geo geodetic coordinates.
     * @param earth_model Earth model.
     * @return state-to-measurement projection matrix.
     */
    H_type create_meas_proj_mtx(const Vector3D & geo, const boost::gregorian::date & day) const;

private:
    static constexpr std::size_t accum_size { 500 };

    void do_reset() override;

    quat::Quaternion do_get_orientation_quaternion() const override;
    Vector3D do_get_gyro_bias() const override;

    void do_set_proc_gyro_std(double std) override;
    void do_set_proc_gyro_bias_std(double std) override;
    void do_set_meas_accel_std(double std) override;
    void do_set_meas_magn_std(double std) override;
    void do_set_init_qs_std(double std) override;
    void do_set_init_qx_std(double std) override;
    void do_set_init_qy_std(double std) override;
    void do_set_init_qz_std(double std) override;
    void do_set_init_bias_std(double std) override;

    double do_get_proc_gyro_std() const override;
    double do_get_proc_gyro_bias_std() const override;
    double do_get_meas_accel_std() const override;
    double do_get_meas_magn_std() const override;
    double do_get_init_qs_std() const override;
    double do_get_init_qx_std() const override;
    double do_get_init_qy_std() const override;
    double do_get_init_qz_std() const override;
    double do_get_init_bias_std() const override;

    const Earth earth_model;            //!< Reference Earth model.
    state_type x;
    P_type P;

    QualityControl<Vector3D> bias_ctrl;
    bool initialized;

    struct
    {

    } params;
};

#endif // KALMANORIENTATIONFILTERBASE_H
