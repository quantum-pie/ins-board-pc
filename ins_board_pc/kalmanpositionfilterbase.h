#ifndef KALMANPOSITIONFILTERBASE_H
#define KALMANPOSITIONFILTERBASE_H

#include "IKalmanPositionFilter.h"

class KalmanPositionFilterBase : virtual public IKalmanPositionFilter
{
public:
    /*!
     * @brief Kalman filter process noise parameters.
     */
    struct ProcessNoiseParams
    {
        double accel_std;       //!< Process noise acceleration standard deviation.
    };

    /*!
     * @brief Kalman filter measurement noise parameters.
     */
    struct MeasurementNoiseParams
    {
        double gps_cep;         //!< measured position CEP (GPS).
        double gps_vel_std;     //!< measured velocity x std.
    };

    /*!
     * @brief Kalman filter initial state estimate covariance parameters.
     */
    struct InitCovParams
    {
        double pos_std;     //!< Initial position estimate standard deviation.
        double vel_std;     //!< Initial velocity estimate standard deviation.
        double accel_std;   //!< Initial acceleration estimate standard deviation.
    };

    /*!
     * @brief Kalman filter parameters structure.
     */
    struct FilterParams
    {
        ProcessNoiseParams proc_params;         //!< Process noise parameters instance.
        MeasurementNoiseParams meas_params;     //!< Measurement noise parameters instance.
        InitCovParams init_params;              //!< Initial state estimate covariance parameters instance.
    };

    explicit KalmanPositionFilterBase(const FilterParams & params);
    ~KalmanPositionFilterBase() override;

    static constexpr std::size_t state_size { 9 };
    static constexpr std::size_t measurement_size { 6 };

    using F_type = StaticMatrix<state_size, state_size>;
    using P_type = F_type;
    using Q_type = F_type;
    using R_type = StaticMatrix<measurement_size, measurement_size>;
    using H_type = StaticMatrix<measurement_size, state_size>;

    /*!
     * @brief Create state transition matrix (F).
     * @param dt time elapsed since the last step.
     * @return state transition matrix.
     */
    F_type create_transition_mtx(double dt) const;

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
     * @param geo geodetic coordinates vector.
     * @return measurement noise covariance matrix.
     */
    R_type create_meas_noise_cov_mtx(const Vector3D & geo) const;

    /*!
     * @brief Create state-to-measurement projection matrix (H).
     * @return state-to-measurement projection matrix.
     */
    H_type create_meas_proj_mtx() const;

private:
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

    FilterParams params; //!< Filter parameters.
};

#endif // KALMANPOSITIONFILTERBASE_H
