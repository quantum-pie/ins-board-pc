/*! \file orientationcomplement.h
  */

#ifndef ORIENTATIONCOMPLEMENT_H
#define ORIENTATIONCOMPLEMENT_H

#include "orientationfilter.h"

/*!
 * \brief Concrete complementary orientation filter class.
 */
class OrientationCF final : public OrientationFilter
{
public:
    /*!
     * \brief Filter parameters structure.
     */
    struct FilterParams
    {
        double static_accel_gain;   //!< Static accelerometer measurements gain.
        double static_magn_gain;    //!< Static magnetometer measurements gain.
        double bias_gain;
        int accum_capacity;         //!< Capacity of measurements accumulator.
    };

    /*!
     * \brief Constructor.
     * \param params filter paramateres structure.
     */
    OrientationCF(const FilterParams & params);

    /*!
     * \brief Destructor.
     */
    ~OrientationCF() override;

    /*!
     * \brief Filter step.
     * \param z filter input reference.
     */
    void step(const FilterInput & z) override;

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
     * \brief Set static accelerometer measurements gain.
     * \param gain static accelerometer measurements gain.
     */
    void set_static_accel_gain(double gain);

    /*!
     * \brief Set static magnetometer measurements gain.
     * \param gain static magnetometer measurements gain.
     */
    void set_static_magn_gain(double gain);

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
    void initialize(const FilterInput & z);

    /*!
     * \brief normalize filter state.
     */
    void normalize_state() override;

private:
    /*!
     * \brief Calculate dynamic accelerometer measurements gain.
     * \param accel measured acceleration.
     */
    double calculate_gain(const NumVector & accel) const;

    NumVector measured_quaternion(const NumVector & accel, const NumVector & magn) const;

    static const int state_size;    //!< State vector size.

    FilterParams params;            //!< Filter parameters structure.
};

#endif
