/*
 * IComplementOrientationFilterAttributes.h
 *
 *      Author: bigaw
 */

#ifndef INCLUDE_ICOMPLEMENTORIENTATIONFILTER_H_
#define INCLUDE_ICOMPLEMENTORIENTATIONFILTER_H_

#include "filtering/public_interfaces/IOrientationFilter.h"
#include "core/IComplementOrientationAttrSetCore.h"
#include "core/IComplementOrientationAttrGetCore.h"

/*!
 * @brief Complementary orientation filter interface.
 */
struct IComplementOrientationFilter : IOrientationFilter,
                                        private IComplementOrientationAttrSetCore,
                                        private IComplementOrientationAttrGetCore
{
    /*!
     * @brief Set static accelerometer measurements gain.
     * @param gain static accelerometer measurements gain.
     */
    void set_static_accel_gain(double gain)
    {
        do_set_static_accel_gain(gain);
    }

    /*!
     * @brief Set static magnetometer measurements gain.
     * @param gain static magnetometer measurements gain.
     */
    void set_static_magn_gain(double gain)
    {
        do_set_static_magn_gain(gain);
    }

    /*!
     * @brief Set bias gain.
     * @param gain bias gain.
     */
    void set_bias_gain(double gain)
    {
        do_set_bias_gain(gain);
    }

    /*!
     * @brief Get static accelerometer measurements gain.
     * @return static accelerometer measurements gain.
     */
    double get_static_accel_gain() const
    {
        return do_get_static_accel_gain();
    }

    /*!
     * @brief Get static magnetometer measurements gain.
     * @return static magnetometer measurements gain.
     */
    double get_static_magn_gain() const
    {
        return do_get_static_magn_gain();
    }

    /*!
     * @brief Get bias gain.
     * @return bias gain.
     */
    double get_bias_gain() const
    {
        return do_get_bias_gain();
    }

    /*!
     * @brief Class destructor.
     */
    ~IComplementOrientationFilter() override = default;
};

#endif /* INCLUDE_ICOMPLEMENTORIENTATIONFILTER_H_ */
