/*
 * IKalmanOrientationFilterAttributes.h
 *
 *      Author: bigaw
 */

#ifndef INCLUDE_IKALMANORIENTATIONFILTER_H_
#define INCLUDE_IKALMANORIENTATIONFILTER_H_

#include "IOrientationFilter.h"

/*!
 * @brief Kalman orientation filte interface.
 */
struct IKalmanOrientationFilter : IOrientationFilter
{
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
};

#endif /* INCLUDE_IKALMANORIENTATIONFILTER_H_ */
