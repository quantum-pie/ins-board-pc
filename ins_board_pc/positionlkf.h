/*
 * positionlkf.h
 *
 *      Author: Ermakov_P
 */

#ifndef INCLUDE_POSITIONLKF_H_
#define INCLUDE_POSITIONLKF_H_

#include "IKalmanPositionFilter.h"
#include "kalmanpositionfilterbase.h"

#include <memory>

/*!
 * @brief Concrete Kalman linear position filter.
 */
class PositionLKF final : virtual public IKalmanPositionFilter,
                          KalmanPositionFilterBase
{
public:
    /*!
     * @brief Class constructor.
     * \param par filter parameters structure.
     */
    explicit PositionLKF(const FilterParams & par);

    /*!
     * @brief Class destructor.
     */
    ~PositionLKF() override;

private:
    /* Interface implementation */
    void do_step(const FilterInput & z) override;
    void do_reset() override;

    Vector3D do_get_cartesian() const override;
    Ellipsoid do_get_ellipsoid() const override;
    Vector3D do_get_velocity() const override;
    Vector3D do_get_acceleration() const override;

    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

#endif /* INCLUDE_POSITIONLKF_H_ */
