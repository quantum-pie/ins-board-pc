/*! \file fullekf.h
  */

#ifndef FULLEKF_H
#define FULLEKF_H

#include "kalmanorientationfilter.h"
#include "kalmanpositionfilter.h"

/*!
 * \brief Concrete Extended Kalman filter for simultaneous orientation and position estimation.
 */
class FullEKF final : public KalmanOrientationFilter, public KalmanPositionFilter
{
public:
    /*!
     * \brief Kalman filter process noise parameters.
     */
    struct ProcessNoiseParams
    {
        double gyro_std;        //!< Process noise gyroscope standard deviation.
        double gyro_bias_std;   //!< Process noise gyroscope bias standard deviation.
        double accel_std;       //!< Process noise acceleration standard deviation.
    };

    /*!
     * \brief Kalman filter measurement noise parameters.
     */
    struct MeasurementNoiseParams
    {
        double accel_std;           //!< accelerometer measurements std.
        double magn_std;            //!< magnetometer measurements std.
        double gps_cep;             //!< measured position CEP (GPS).
        double gps_vel_std;     //!< measured velocity std.
    };

    /*!
     * \brief Kalman filter initial state estimate covariance parameters.
     */
    struct InitCovParams
    {
        double qs_std;      //!< Initial qs estimate standard deviation.
        double qx_std;      //!< Initial qx estimate standard deviation.
        double qy_std;      //!< Initial qy estimate standard deviation.
        double qz_std;      //!< Initial qz estimate standard deviation.
        double bias_std;    //!< Initial gyro bias estimate standard deviation.
        double pos_std;     //!< Initial position estimate standard deviation.
        double vel_std;     //!< Initial velocity estimate standard deviation.
        double accel_std;   //!< Initial acceleration estimate standard deviation.
    };

    /*!
     * \brief Kalman filter parameters structure.
     */
    struct FilterParams
    {
        ProcessNoiseParams proc_params;     //!< Process noise parameters instance.
        MeasurementNoiseParams meas_params; //!< Measurement noise parameters instance.
        InitCovParams init_params;          //!< Initial state estimate covariance parameters instance.
        int accum_capacity;                 //!< Capacity of filter input accumulator.
        int track_history;
    };

    /*!
     * \brief Constructor.
     * \param params filter parameters.
     */
    FullEKF(const FilterParams & params);

    /*!
     * \brief Destructor.
     */
    ~FullEKF() override;

    /*!
     * \brief Filter step.
     * \param z filter input reference.
     */
    void step(const FilterInput & z) override;

    /*!
     * \brief Reset filter.
     */
    void reset() override;

    /*!
     * \brief Get current orientation quaternion.
     * \return vector representing quaternion.
     */
    NumVector get_orientation_quaternion() const override;

    /*!
     * \brief Get current gyroscope bias.
     * \return gyroscope bias vector.
     */
    NumVector get_gyro_bias() const override;

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
     * \brief Set process noise gyroscope standard deviation.
     * \param std standard deviation.
     */
    void set_proc_gyro_std(double std) override;

    /*!
     * \brief Set process noise gyroscope bias standard deviation.
     * \param std standard deviation.
     */
    void set_proc_gyro_bias_std(double std) override;

    /*!
     * \brief Set process noise acceleration standard deviation.
     * \param std standard deviation.
     */
    void set_proc_accel_std(double std) override;

    /*!
     * \brief Set accelerometer measurements standard deviation.
     * \param std standard deviation.
     */
    void set_meas_accel_std(double std) override;

    /*!
     * \brief Set magnetometer measurements standard deviation.
     * \param std standard deviation.
     */
    void set_meas_magn_std(double std) override;

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
     * \brief Set initial qs estimate standard deviation.
     * \param std standard deviation.
     */
    void set_init_qs_std(double std) override;

    /*!
     * \brief Set initial qx estimate standard deviation.
     * \param std standard deviation.
     */
    void set_init_qx_std(double std) override;

    /*!
     * \brief Set initial qy estimate standard deviation.
     * \param std standard deviation.
     */
    void set_init_qy_std(double std) override;

    /*!
     * \brief Set initial qz estimate standard deviation.
     * \param std standard deviation.
     */
    void set_init_qz_std(double std) override;

    /*!
     * \brief Set initial bias estimate standard deviation.
     * \param std standard deviation.
     */
    void set_init_bias_std(double std) override;

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
     * \brief Accumulate filter input.
     * \param z filter input reference.
     */
    void accumulate(const FilterInput & z) override;

    /*!
     * \brief Initialize filter.
     * \param z filter input reference.
     */
    void initialize(const FilterInput & z) override;

    /*!
     * \brief normalize filter state.
     */
    void normalize_state() override;

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
     * \param alt geodetic altitude above ellipsoid.
     * \param day current date.
     * \return measurement noise covariance matrix.
     */
    NumMatrix create_meas_noise_cov_mtx(const NumVector & geo, QDate day) const;

    /*!
     * \brief Create state-to-measurement projection matrix (H).
     * \param lat geodetic latitude.
     * \param lon geodetic longitude.
     * \param alt geodetic altitude above ellipsoid.
     * \param day current date.
     * \param v predicted velocity vector.
     * \return state-to-measurement projection matrix.
     */
    NumMatrix create_meas_proj_mtx(const NumVector & geo, QDate day) const;

    /*!
     * \brief Map state quaternion to accelerometer measurements.
     * \param orientation_quat state quaternion.
     * \param acceleration state cartesian acceleration.
     * \param lat geodetic latitude.
     * \param lon geodetic longitude.
     * \param alt geodetic altitude above ellipsoid.
     * \param[out] ax x-componet of accelerometer measurements.
     * \param[out] ay y-componet of accelerometer measurements.
     * \param[out] az z-componet of accelerometer measurements.
     */
    NumVector calculate_accelerometer(const NumVector & orientation_quat, const NumVector & acceleration,
                                 const NumVector & geo) const;

    /*!
     * \brief Map state quaternion to magnetometer measurements.
     * \param orientation_quat state quaternion.
     * \param lat geodetic latitude.
     * \param lon geodetic longitude.
     * \param alt geodetic altitude above ellipsoid.
     * \param day current date.
     * \param[out] mx x-componet of magnetometer measurements.
     * \param[out] my y-componet of magnetometer measurements.
     * \param[out] mz z-componet of magnetometer measurements.
     */
    NumVector calculate_magnetometer(const NumVector & orientation_quat,
                                const NumVector & geo, QDate day) const;

    static const int state_size;        //!< Size of state vector.
    static const int measurement_size;  //!< Size of measurements vector.

    NumMatrix P;                        //!< state estimate covariance matrix.
    FilterParams params;                //!< Filter parameters instance.
};

#endif // FULLEKF_H
