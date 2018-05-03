/*! \file poskalman.h
  */

#ifndef POSITIONLKF_H
#define POSITIONLKF_H

#include "kalmanpositionfilter.h"

/*!
 * \brief Concrete Kalman position estimator.
 */
class PositionLKF final : public KalmanPositionFilter
{
public:
    /*!
     * \brief Kalman filter process noise parameters.
     */
    struct ProcessNoiseParams
    {
        double accel_std;       //!< Process noise acceleration standard deviation.
    };

    /*!
     * \brief Kalman filter measurement noise parameters.
     */
    struct MeasurementNoiseParams
    {
        double gps_cep;           //!< measured position CEP (GPS).
        double gps_vel_std;     //!< measured velocity x std.
    };

    /*!
     * \brief Kalman filter initial state estimate covariance parameters.
     */
    struct InitCovParams
    {
        double pos_std;     //!< Initial position estimate standard deviation.
        double vel_std;     //!< Initial velocity estimate standard deviation.
        double accel_std;   //!< Initial acceleration estimate standard deviation.
    };

    /*!
     * \brief Kalman filter parameters structure.
     */
    struct FilterParams
    {
        ProcessNoiseParams proc_params;         //!< Process noise parameters instance.
        MeasurementNoiseParams meas_params;     //!< Measurement noise parameters instance.
        InitCovParams init_params;              //!< Initial state estimate covariance parameters instance.
        int track_history;
    };

    /*!
     * \brief Constructor.
     * \param params filter parameters.
     */
    PositionLKF(const FilterParams & params);

    /*!
     * \brief Destructor.
     */
    ~PositionLKF() override;

    /*!
     * \brief Filter step.
     * \param z filter input reference.
     */
    void step(const FilterInput & z) override;

    void sim_step(const FilterInput & z);
    void sim_initialize(const FilterInput & z);
    void sim_update(const FilterInput & z);

    void bypass_step(const FilterInput & z);

    /*!
     * \brief Get current position vector.
     * \return position vector.
     */
    NumVector get_position() const override;

    /*!
     * \brief Get current velocity vector.
     * \return velocity vector.
     */
    NumVector get_velocity() const override;

    /*!
     * \brief Get current acceleration vector.
     * \return acceleration vector.
     */
    NumVector get_acceleration() const override;

    /*!
     * \brief Set process noise acceleration standard deviation.
     * \param std standard deviation.
     */
    void set_proc_accel_std(double std) override;

    /*!
     * \brief Set measured position CEP.
     * \param std GPS CEP parameter.
     */
    void set_meas_pos_std(double std) override;

    /*!
     * \brief Set measured velocity standard deviation.
     * \param std standard deviation.
     */
    void set_meas_vel_std(double std) override;

    /*!
     * \brief Set initial position estimate standard deviation.
     * \param std standard deviation.
     */
    void set_init_pos_std(double std) override;

    /*!
     * \brief Set initial velocity estimate standard deviation.
     * \param std standard deviation.
     */
    void set_init_vel_std(double std) override;

    /*!
     * \brief Set initial acceleration estimate standard deviation.
     * \param std standard deviation.
     */
    void set_init_accel_std(double std) override;

protected:
    /*!
     * \brief Update filter state.
     * \param z filter input reference.
     */
    void update(const FilterInput & z) override;

    /*!
     * \brief Initialize filter.
     * \param z filter input reference.
     */
    void initialize(const FilterInput & z) override;

private:
    /*!
     * \brief Create state transition matrix (F).
     * \param z filter input reference.
     * \return state transition matrix.
     */
    NumMatrix create_transition_mtx(const FilterInput & z) const;

    /*!
     * \brief Create process noise covariance matrix (Q).
     * \param dt time elapsed since the last measurement.
     * \return process noise covariance matrix.
     */
    NumMatrix create_proc_noise_cov_mtx(double dt) const;

    /*!
     * \brief Create measurement noise covariance matrix (R).
     * \param lat geodetic latitude.
     * \param lon geodetic longitude.
     * \return measurement noise covariance matrix.
     */
    NumMatrix create_meas_noise_cov_mtx(const NumVector & geo) const;

    /*!
     * \brief Create state-to-measurement projection matrix (H).
     * \param v predicted velocity vector.
     * \return state-to-measurement projection matrix.
     */
    NumMatrix create_meas_proj_mtx() const;

    void initialize_init_covar();

    static const int state_size;        //!< Size of state vector.
    static const int measurement_size;  //!< Size of measurements vector.

    NumMatrix P;                        //!< state estimate covariance matrix.
    FilterParams params;                //!< Filter parameters instance.
};

#endif // POSITIONLKF_H
