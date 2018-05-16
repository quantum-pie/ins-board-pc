/*
 * positionsim.h
 *
 *      Author: Ermakov_P
 */

#ifndef INCLUDE_POSITIONSIM_H_
#define INCLUDE_POSITIONSIM_H_

#include "filtering/public_interfaces/IPositionFilter.h"

#include <memory>

/*!
 * @brief Concrete Kalman linear position filter simulator.
 */
class PositionSim final : public IPositionFilter
{
public:
    /*!
     * @brief Class constructor.
     * @param params filter parameters.
     */
    PositionSim();

    /*!
     * @brief Class destructor.
     */
    ~PositionSim() override = default;

	/*!
	 * @brief Set start track angle.
	 * @param radians track angle.
	 */
    void set_initial_track(double radians);

    /*!
     * @brief Set movement speed.
     * @param ms speed in m/s.
     */
    void set_speed(double ms);

    /*!
     * @brief Get current start track angle.
     * @return track angle.
     */
    double get_initial_track() const;

    /*!
     * @brief Get current movement speed.
     * @return movement speed in m/s.
     */
    double get_speed() const;

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

#endif /* INCLUDE_POSITIONSIM_H_ */
