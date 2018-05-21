/*
 * positionsim.h
 *
 *      Author: Ermakov_P
 */

#ifndef INCLUDE_POSITIONSIM_H_
#define INCLUDE_POSITIONSIM_H_

#include "filtering/public_interfaces/ISimPositionFilter.h"

#include <memory>

/*!
 * @brief Concrete Kalman linear position filter simulator.
 */
class PositionSim final : public ISimPositionFilter
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

private:
    /* IPositionFilter interface implementation */
    void do_step(const FilterInput & z) override;
    void do_reset() override;

    Vector3D do_get_cartesian() const override;
    Ellipsoid do_get_ellipsoid() const override;
    Vector3D do_get_velocity() const override;
    Vector3D do_get_acceleration() const override;

    void do_set_initial_track(double radians) override;
    void do_set_speed(double ms) override;
    double do_get_initial_track() const override;
    double do_get_speed() const override;

    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

#endif /* INCLUDE_POSITIONSIM_H_ */
