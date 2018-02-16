/*! \file abstractorientationfilter.h
  */

#ifndef ABSTRACTORIENTATIONFILTER_H
#define ABSTRACTORIENTATIONFILTER_H

#include "abstractfilter.h"
#include "qualitycontrol.h"

/*!
 * \brief Abstract body orientation filter.
 *
 * Base class for all filters capable of estimating rigid body orientation.
 */
class AbstractOrientationFilter : public virtual AbstractFilter
{
public:
    /*!
     * \brief Constructor.
     * \param accum_capacity capacity of input accumulator.
     */
    AbstractOrientationFilter(int accum_capacity)
        : AbstractFilter(),
          bias_x_ctrl(accum_capacity), bias_y_ctrl(accum_capacity), bias_z_ctrl(accum_capacity)
    {
        reset_this();
    }

    /*!
     * \brief Destructor.
     */
    ~AbstractOrientationFilter() override {}

    /*!
     * \brief Reset filter.
     */
    void reset() override
    {
        AbstractFilter::reset();
        reset_this();
    }

    /*!
     * \brief Get current orientation quaternion.
     * \return vector representing quaternion.
     */
    virtual NumVector get_orientation_quaternion() const = 0;

    /*!
     * \brief Get current gyroscope bias.
     * \return gyroscope bias vector.
     */
    virtual NumVector get_gyro_bias() const = 0;

    /*!
     * \brief Get current Euler angles.
     * \param[out] roll current roll angle.
     * \param[out] pitch current pitch angle.
     * \param[out] yaw current yaw angle.
     */
    virtual void get_rpy(double & roll, double & pitch, double & yaw) const = 0;

protected:
    /*!
     * \brief Accumulate filter input.
     * \param z filter input reference.
     */
    void accumulate(const FilterInput & z) override
    {
        bias_x_ctrl.update(z.w[0]);
        bias_y_ctrl.update(z.w[1]);
        bias_z_ctrl.update(z.w[2]);
    }

    /*!
     * \brief Check gyroscope bias estimation status.
     * \return true if bias is estimated
     */
    bool bias_estimated() const
    {
        return bias_x_ctrl.is_saturated() &&
                bias_y_ctrl.is_saturated() &&
                bias_z_ctrl.is_saturated();
    }

    /*!
     * \brief normalize filter state.
     */
    virtual void normalize_state() = 0;

    QualityControl bias_x_ctrl; //!< x-axis bias controller.
    QualityControl bias_y_ctrl; //!< y-axis bias controller.
    QualityControl bias_z_ctrl; //!< z-axis bias controller.

private:
    /*!
     * \brief Reset implemented part.
     */
    void reset_this()
    {
        bias_x_ctrl.reset();
        bias_y_ctrl.reset();
        bias_z_ctrl.reset();
    }
};

#endif // ABSTRACTORIENTATIONFILTER_H
