/*
 * positionbypass.h
 *
 *      Author: bigaw
 */
#ifndef INCLUDE_POSITIONBYPASS_H_
#define INCLUDE_POSITIONBYPASS_H_

#include "filtering/public_interfaces/IPositionFilter.h"

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
    ~PositionBypass() override = default;

private:
    /* IPositionFilter interface implementation */
    void do_step(const FilterInput & z) override;
    void do_reset() override;

    Vector3D do_get_cartesian() const override;
    Ellipsoid do_get_ellipsoid() const override;
    Vector3D do_get_velocity() const override;
    Vector3D do_get_acceleration() const override;

    Vector3D pos;
    Vector3D vel;
};

#endif /* INCLUDE_POSITIONBYPASS_H_ */
