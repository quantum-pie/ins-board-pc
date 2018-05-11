/*! \file orientationekf.h
  */

#ifndef ORIENTATIONEKF_H
#define ORIENTATIONEKF_H

#include "IKalmanOrientationFilter.h"
#include "kalmanorientationfilterbase.h"
#include "qualitycontrol.h"
#include "earth.h"

/*!
 * @brief Concrete Kalman filter for orientation estimation.
 */
class OrientationEKF final : public virtual IKalmanOrientationFilter,
                             KalmanOrientationFilterBase
{
public:
    /*!
     * @brief Constructor.
     * @param params filter parameters.
     */
    explicit OrientationEKF(const FilterParams & par);

    /*!
     * \brief Destructor.
     */
    ~OrientationEKF() override;

private:
    /* Interfaces implementation */
    void do_step(const FilterInput & z) override;
    void do_reset() override;

    quat::Quaternion do_get_orientation_quaternion() const override;
    Vector3D do_get_gyro_bias() const override;

    /*!
     * @brief Step of initialized filter.
     * @param z filter input.
     */
    void step_initialized(const FilterInput & z);

    /*!
     * @brief Step of uninitialized filter.
     * @param z filter input.
     */
    void step_uninitialized(const FilterInput & z);

    /*!
     * @brief Initialize filter.
     * @param z filter input.
     */
    void initialize(const FilterInput & z);

    /*!
     * @brief normalize filter state.
     */
    void normalize_state();

     /* Useful aliases */
    using state_type = StaticVector<state_size>;
    using meas_type = StaticVector<measurement_size>;

    static constexpr std::size_t buffer_size { 1000 };

    bool is_initialized;                //!< Filter is initialized flag.

    const Earth earth_model;            //!< Reference Earth model.
    QualityControl<Vector3D> bias_ctrl; //!< Gyroscope bias controller.
    state_type x;                       //!< State vector.
    P_type P;                           //!< State estimate covariance matrix.
};

#endif // ORIENTATIONEKF_H
