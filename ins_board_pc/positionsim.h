/*
 * positionsim.h
 *
 *      Author: Ermakov_P
 */

#ifndef INCLUDE_POSITIONSIM_H_
#define INCLUDE_POSITIONSIM_H_

#include "IPositionFilter.h"

/*!
 * @brief Concrete Kalman linear position filter simulator.
 */
class PositionSim final : public IPositionFilter
{
public:
    /*!
     * @brief Simulator parameters structure.
     */
    struct FilterParams
    {
        double initial_track;       //!< Start track angle.
        double speed;               //!< Movement speed.
    };

    /*!
     * @brief Class constructor.
     * @param params filter parameters.
     */
    explicit PositionSim(const FilterParams & params);

    /*!
     * @brief Class destructor.
     */
    ~PositionSim() override;

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

    /* IPositionFilter interface implementation */
    void step(const FilterInput & z) override;
    void reset() override;

	Vector3D get_cartesian() const override;
    Ellipsoid get_ellipsoid() const override;
	Vector3D get_velocity() const override;
	Vector3D get_acceleration() const override;

private:
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

    static constexpr int state_size { 9 };        	//!< Size of state vector.
    using state_type = StaticVector<state_size>;

    bool is_initialized;                            //!< Filter is initialized flag.
    FilterParams params;                            //!< Filter parameters instance.
    state_type x;                                   //!< Filter state.
};

#endif /* INCLUDE_POSITIONSIM_H_ */
