/*
 * positionbypass.h
 *
 *      Author: bigaw
 */
#ifndef INCLUDE_POSITIONBYPASS_H_
#define INCLUDE_POSITIONBYPASS_H_

#include "IPositionFilter.h"
#include "IFilter.h"

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

    /* IPositionFilter interface implementation */
    void step(const FilterInput & z) override;
    void reset() override;

	Vector3D get_cartesian() const override;
    Ellipsoid get_ellipsoid() const override;
	Vector3D get_velocity() const override;
	Vector3D get_acceleration() const override;

private:
    static constexpr int state_size { 9 };        	//!< Size of state vector.
    using state_type = StaticVector<state_size>;
    state_type x;									//!< State vector.
};

#endif /* INCLUDE_POSITIONBYPASS_H_ */
