/*! \file abstractkalmanpositionfilter.h
  */

#ifndef ABSTRACTKALMANPOSITIONFILTER_H
#define ABSTRACTKALMANPOSITIONFILTER_H

#include "abstractpositionfilter.h"

/*!
 * \brief Abstract point position Kalman filter.
 *
 * Base class for all Kalman filters capable of estimating point position.
 */
class AbstractKalmanPositionFilter : public AbstractPositionFilter
{
public:
    /*!
     * \brief Constructor.
     */
    AbstractKalmanPositionFilter() : AbstractPositionFilter() {}

    /*!
     * \brief Destructor.
     */
    ~AbstractKalmanPositionFilter() override {}

    /*!
     * \brief Set process noise acceleration standard deviation.
     * \param std standard deviation.
     */
    virtual void set_proc_accel_std(double std) = 0;

    /*!
     * \brief Set measured position CEP.
     * \param std GPS CEP parameter.
     */
    virtual void set_meas_pos_std(double std) = 0;

    /*!
     * \brief Set measured velocity standard deviation.
     * \param std standard deviation.
     */
    virtual void set_meas_vel_std(double std) = 0;

    /*!
     * \brief Set initial position estimate standard deviation.
     * \param std standard deviation.
     */
    virtual void set_init_pos_std(double std) = 0;

    /*!
     * \brief Set initial velocity estimate standard deviation.
     * \param std standard deviation.
     */
    virtual void set_init_vel_std(double std) = 0;

    /*!
     * \brief Set initial acceleration estimate standard deviation.
     * \param std standard deviation.
     */
    virtual void set_init_accel_std(double std) = 0;
};

#endif // ABSTRACTKALMANPOSITIONFILTER_H
