/*! \file abstractkalmanorientationfilter.h
  */

#ifndef ABSTRACTKALMANORIENTATIONFILTER_H
#define ABSTRACTKALMANORIENTATIONFILTER_H

#include "abstractorientationfilter.h"

/*!
 * \brief Abstract body orientation Kalman filter.
 *
 * Base class for all Kalman filters capable of estimating rigid body orientation.
 */
class AbstractKalmanOrientationFilter : public AbstractOrientationFilter
{
public:
    /*!
     * \brief Constructor.
     * \param accum_capacity capacity of input accumulator.
     */
    AbstractKalmanOrientationFilter(int accum_capacity)
        : AbstractOrientationFilter(accum_capacity) {}

    /*!
     * \brief Destructor.
     */
    ~AbstractKalmanOrientationFilter() override {}

    /*!
     * \brief Set process noise gyroscope standard deviation.
     * \param std standard deviation.
     */
    virtual void set_proc_gyro_std(double std) = 0;

    /*!
     * \brief Set process noise gyroscope bias standard deviation.
     * \param std standard deviation.
     */
    virtual void set_proc_gyro_bias_std(double std) = 0;

    /*!
     * \brief Set accelerometer measurements standard deviation.
     * \param std standard deviation.
     */
    virtual void set_meas_accel_std(double std) = 0;

    /*!
     * \brief Set magnetometer measurements standard deviation.
     * \param std standard deviation.
     */
    virtual void set_meas_magn_std(double std) = 0;

    /*!
     * \brief Set initial qs estimate standard deviation.
     * \param std standard deviation.
     */
    virtual void set_init_qs_std(double std) = 0;

    /*!
     * \brief Set initial qx estimate standard deviation.
     * \param std standard deviation.
     */
    virtual void set_init_qx_std(double std) = 0;

    /*!
     * \brief Set initial qy estimate standard deviation.
     * \param std standard deviation.
     */
    virtual void set_init_qy_std(double std) = 0;

    /*!
     * \brief Set initial qz estimate standard deviation.
     * \param std standard deviation.
     */
    virtual void set_init_qz_std(double std) = 0;

    /*!
     * \brief Set initial bias estimate standard deviation.
     * \param std standard deviation.
     */
    virtual void set_init_bias_std(double std) = 0;
};

#endif // ABSTRACTKALMANORIENTATIONFILTER_H
