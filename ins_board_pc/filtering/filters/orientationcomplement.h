/*! \file orientationcomplement.h
  */

#ifndef ORIENTATIONCOMPLEMENT_H
#define ORIENTATIONCOMPLEMENT_H

#include "filtering/public_interfaces/IComplementOrientationFilter.h"

#include <memory>

/*!
 * @brief Concrete complementary orientation filter class.
 */
class OrientationCF final : public IComplementOrientationFilter
{
public:
    /*!
     * @brief Class constructor.
     */
    OrientationCF();

    /*!
     * @brief Class destructor.
     */
    ~OrientationCF() override;

private:
    /* IComplementOrientationFilter interface implementation */
    void do_step(const FilterInput & z) override;
    void do_reset() override;

    quat::Quaternion do_get_orientation_quaternion() const override;
    Vector3D do_get_gyro_bias() const override;

    void do_set_static_accel_gain(double gain) override;
    void do_set_static_magn_gain(double gain) override;
    void do_set_bias_gain(double gain) override;

    double do_get_static_accel_gain() const override;
    double do_get_static_magn_gain() const override;
    double do_get_bias_gain() const override;

    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

#endif
