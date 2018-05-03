/*
 * IComplementOrientationFilterAttributes.h
 *
 *      Author: bigaw
 */

#ifndef INCLUDE_ICOMPLEMENTORIENTATIONFILTER_H_
#define INCLUDE_ICOMPLEMENTORIENTATIONFILTER_H_

#include "IOrientationFilter.h"

/*!
 * @brief Complementary orientation filter interface.
 */
struct IComplementOrientationFilter : IOrientationFilter
{
    /*!
     * @brief Class destructor.
     */
    ~IComplementOrientationFilter() override = default;

    /*!
     * @brief Set static accelerometer measurements gain.
     * @param gain static accelerometer measurements gain.
     */
    virtual void set_static_accel_gain(double gain) = 0;

    /*!
     * @brief Set static magnetometer measurements gain.
     * @param gain static magnetometer measurements gain.
     */
    virtual void set_static_magn_gain(double gain) = 0;

    /*!
     * @brief Set bias gain.
     * @param gain bias gain.
     */
    virtual void set_bias_gain(double gain) = 0;

    /*!
     * @brief Get static accelerometer measurements gain.
     * @return static accelerometer measurements gain.
     */
    virtual double get_static_accel_gain() const = 0;

    /*!
     * @brief Get static magnetometer measurements gain.
     * @return static magnetometer measurements gain.
     */
    virtual double get_static_magn_gain() const = 0;

    /*!
     * @brief Get bias gain.
     * @return bias gain.
     */
    virtual double get_bias_gain() const = 0;
};

#endif /* INCLUDE_ICOMPLEMENTORIENTATIONFILTER_H_ */
