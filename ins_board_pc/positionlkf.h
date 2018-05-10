/*
 * positionlkf.h
 *
 *      Author: Ermakov_P
 */

#ifndef INCLUDE_POSITIONLKF_H_
#define INCLUDE_POSITIONLKF_H_

#include "kalmanpositionfilterbase.h"

/*!
 * @brief Concrete Kalman linear position filter.
 */
class PositionLKF final : public KalmanPositionFilterBase
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

    /* Interface implementation */
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

    /* Useful aliases */
    using state_type = StaticVector<state_size>;
    using meas_type = StaticVector<measurement_size>;

    bool is_initialized;                //!< Filter is initialized flag.

    state_type x;                       //!< State vector.
    P_type P;                        	//!< State estimate covariance matrix.
};

#endif /* INCLUDE_POSITIONLKF_H_ */
