/*
 * IKalmanOrientationFilterAttributes.h
 *
 *      Author: bigaw
 */

#ifndef INCLUDE_IKALMANORIENTATIONFILTER_H_
#define INCLUDE_IKALMANORIENTATIONFILTER_H_

#include "IOrientationFilter.h"

/*!
 * @brief Kalman orientation filter interface.
 */
struct IKalmanOrientationFilter : IOrientationFilter
{
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
     * @brief Class destructor.
     */
    ~IKalmanOrientationFilter() override = default;

    /*!
     * @brief Set process noise gyroscope standard deviation.
     * @param std standard deviation.
     */
    virtual void set_proc_gyro_std(double std) = 0;

    /*!
     * @brief Set process noise gyroscope bias standard deviation.
     * @param std standard deviation.
     */
    virtual void set_proc_gyro_bias_std(double std) = 0;

    /*!
     * @brief Set accelerometer measurements standard deviation.
     * @param std standard deviation.
     */
    virtual void set_meas_accel_std(double std) = 0;

    /*!
     * @brief Set magnetometer measurements standard deviation.
     * @param std standard deviation.
     */
    virtual void set_meas_magn_std(double std) = 0;

    /*!
     * @brief Set initial qs estimate standard deviation.
     * @param std standard deviation.
     */
    virtual void set_init_qs_std(double std) = 0;

    /*!
     * @brief Set initial qx estimate standard deviation.
     * @param std standard deviation.
     */
    virtual void set_init_qx_std(double std) = 0;

    /*!
     * @brief Set initial qy estimate standard deviation.
     * @param std standard deviation.
     */
    virtual void set_init_qy_std(double std) = 0;

    /*!
     * @brief Set initial qz estimate standard deviation.
     * @param std standard deviation.
     */
    virtual void set_init_qz_std(double std) = 0;

    /*!
     * @brief Set initial bias estimate standard deviation.
     * @param std standard deviation.
     */
    virtual void set_init_bias_std(double std) = 0;

    /*!
     * @brief Get process noise gyroscope bias standard deviation.
     * @return standard deviation.
     */
    virtual double get_proc_gyro_std() const = 0;

    /*!
     * @brief Get process noise gyroscope bias standard deviation.
     * @return standard deviation.
     */
    virtual double get_proc_gyro_bias_std() const = 0;

    /*!
     * @brief Get accelerometer measurements standard deviation.
     * @return standard deviation.
     */
    virtual double get_meas_accel_std() const = 0;

    /*!
     * @brief Get magnetometer measurements standard deviation.
     * @return standard deviation.
     */
    virtual double get_meas_magn_std() const = 0;

    /*!
     * @brief Get initial qs estimate standard deviation.
     * @return standard deviation.
     */
    virtual double get_init_qs_std() const = 0;

    /*!
     * @brief Get initial qx estimate standard deviation.
     * @return standard deviation.
     */
    virtual double get_init_qx_std() const = 0;

    /*!
     * @brief Get initial qy estimate standard deviation.
     * @return standard deviation.
     */
    virtual double get_init_qy_std() const = 0;

    /*!
     * @brief Get initial qz estimate standard deviation.
     * @return standard deviation.
     */
    virtual double get_init_qz_std() const = 0;

    /*!
     * @brief Get initial bias estimate standard deviation.
     * @return standard deviation.
     */
    virtual double get_init_bias_std() const = 0;
};

#endif /* INCLUDE_IKALMANORIENTATIONFILTER_H_ */
