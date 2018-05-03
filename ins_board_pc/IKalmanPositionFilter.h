/*
 * IKalmanPositionFilterAttributes.h
 *
 *      Author: bigaw
 */

#ifndef INCLUDE_IKALMANPOSITIONFILTERATTRIBUTES_H_
#define INCLUDE_IKALMANPOSITIONFILTERATTRIBUTES_H_

#include "IPositionFilter.h"

/*!
 * @brief Kalman position filter interface.
 */
struct IKalmanPositionFilter : IPositionFilter
{
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
     * @brief Class destructor.
     */
    ~IKalmanPositionFilter() override = default;

    /*!
     * @brief Set process noise acceleration standard deviation.
     * @param std standard deviation.
     */
    virtual void set_proc_accel_std(double std) = 0;

    /*!
     * @brief Set measured position CEP.
     * @param std GPS CEP parameter.
     */
    virtual void set_meas_pos_std(double std) = 0;

    /*!
     * @brief Set measured velocity standard deviation.
     * @param std standard deviation.
     */
    virtual void set_meas_vel_std(double std) = 0;

    /*!
     * @brief Set initial position estimate standard deviation.
     * @param std standard deviation.
     */
    virtual void set_init_pos_std(double std) = 0;

    /*!
     * @brief Set initial velocity estimate standard deviation.
     * @param std standard deviation.
     */
    virtual void set_init_vel_std(double std) = 0;

    /*!
     * @brief Set initial acceleration estimate standard deviation.
     * @param std standard deviation.
     */
    virtual void set_init_accel_std(double std) = 0;

    /*!
     * @brief Get process noise acceleration standard deviation.
     * @return standard deviation.
     */
    virtual double get_proc_accel_std() const = 0;

    /*!
     * @brief Get measured position CEP.
     * @return GPS CEP parameter.
     */
    virtual double get_meas_pos_std() const = 0;

    /*!
     * @brief Get measured velocity standard deviation.
     * @return standard deviation.
     */
    virtual double get_meas_vel_std() const = 0;

    /*!
     * @brief Get initial position estimate standard deviation.
     * @return standard deviation.
     */
    virtual double get_init_pos_std() const = 0;

    /*!
     * @brief Get initial velocity estimate standard deviation.
     * @return standard deviation.
     */
    virtual double get_init_vel_std() const = 0;

    /*!
     * @brief Get initial acceleration estimate standard deviation.
     * @return standard deviation.
     */
    virtual double get_init_accel_std() const = 0;
};

#endif /* INCLUDE_IKALMANPOSITIONFILTERATTRIBUTES_H_ */
