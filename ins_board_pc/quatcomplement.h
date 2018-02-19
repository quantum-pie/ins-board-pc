/*! \file quatcomplement.h
  */

#ifndef QUATCOMPLEMENT_H
#define QUATCOMPLEMENT_H

#include "abstractorientationfilter.h"

/*!
 * \brief Concrete complementary orientation filter class.
 */
class QuaternionComplement final : public AbstractOrientationFilter
{
public:
    /*!
     * \brief Filter parameters structure.
     */
    struct FilterParams
    {
        double static_accel_gain;   //!< Static accelerometer measurements gain.
        double static_magn_gain;    //!< Static magnetometer measurements gain.
        int accum_capacity;         //!< Capacity of measurements accumulator.
    };

    /*!
     * \brief Constructor.
     * \param params filter paramateres structure.
     */
    QuaternionComplement(const FilterParams & params);

    /*!
     * \brief Destructor.
     */
    ~QuaternionComplement() override;

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
     * \brief Get current Euler angles.
     * \param[out] roll current roll angle.
     * \param[out] pitch current pitch angle.
     * \param[out] yaw current yaw angle.
     */
    void get_rpy(double & roll, double & pitch, double & yaw) const override;

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

    static const int state_size;    //!< State vector size.

    FilterParams params;            //!< Filter parameters structure.
};

#endif
