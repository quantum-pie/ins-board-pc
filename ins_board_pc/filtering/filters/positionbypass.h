/*
 * positionbypass.h
 *
 *      Author: bigaw
 */
#ifndef INCLUDE_POSITIONBYPASS_H_
#define INCLUDE_POSITIONBYPASS_H_

#include "filtering/public_interfaces/IPositionFilter.h"

#include <memory>

// TODO refactor

/*!
 * @brief Concrete Kalman linear position filter bypass.
 */
class PositionBypass final : public IPositionFilter
{
public:
    /*!
     * @brief Class constructor.
     */
	PositionBypass();

    /*!
     * @brief Class destructor.
     */
    ~PositionBypass() override;

private:
    /* IPositionFilter interface implementation */
    void do_step(const FilterInput & z) override;
    void do_reset() override;

    Vector3D do_get_cartesian() const override;
    Ellipsoid do_get_ellipsoid() const override;
    Vector3D do_get_velocity() const override;
    Vector3D do_get_acceleration() const override;

    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

#endif /* INCLUDE_POSITIONBYPASS_H_ */
