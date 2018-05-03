/*! \file positionfilter.h
  */

#ifndef POSITIONFILTER_H
#define POSITIONFILTER_H

#include "filter.h"
#include "wmmwrapper.h"
#include "qualitycontrol.h"

#include <QtMath>

/*!
 * \brief Abstract point position filter.
 *
 * Base class for all filters capable of estimating point position.
 */
class PositionFilter : public virtual Filter
{
public:
    /*!
     * \brief Constructor.
     */
    PositionFilter(int track_history)
        : Filter(),
        vx_ctrl(track_history), vy_ctrl(track_history), vz_ctrl(track_history)
    {}

    /*!
     * \brief Destructor.
     */
    ~PositionFilter() override {}

    void update(const FilterInput &) override
    {
        NumVector vel = this->get_velocity();
        vx_ctrl.update(vel[0]);
        vy_ctrl.update(vel[1]);
        vz_ctrl.update(vel[2]);
    }

    /*!
     * \brief Reset filter.
     */
    void reset() override
    {
        Filter::reset();
        reset_this();
    }

    /*!
     * \brief Get current position vector in Earth-centered, Earth-fixed system.
     * \return position vector.
     */
    virtual NumVector get_position() const = 0;

    /*!
     * \brief Get current position vector in East-North-Up system.
     * \return position vector.
     */
    NumVector get_position_enu()
    {
        return get_position_enu(this->get_position());
    }

    /*!
     * \brief Get arbitrary position vector in East-North-Up system.
     * \param pos position vector.
     * \return position vector.
     */
    NumVector get_position_enu(const NumVector & pos) const
    {
        NumMatrix Cel = WrapperWMM::instance().geodetic_to_dcm(start_geo);
        return Cel * (pos - start_ecef);
    }

    /*!
     * \brief Get heading to the North.
     * \return heading angle.
     */
    double get_track_angle() const
    {
        NumMatrix Cel = WrapperWMM::instance().geodetic_to_dcm(get_geodetic());
        NumVector mean_vel(3);
        mean_vel[0] = vx_ctrl.get_mean();
        mean_vel[1] = vy_ctrl.get_mean();
        mean_vel[2] = vz_ctrl.get_mean();
        NumVector local_vel = Cel * mean_vel;
        return qAtan2(local_vel[0], local_vel[1]);
    }

    double get_ground_speed() const
    {
        NumMatrix Cel = WrapperWMM::instance().geodetic_to_dcm(get_geodetic());
        NumVector mean_vel(3);
        NumVector local_vel = Cel * this->get_velocity();
        return qSqrt(local_vel[0] * local_vel[0] + local_vel[1] * local_vel[1]) * ms_to_knots;
    }

    /*!
     * \brief Get current velocity vector.
     * \return velocity vector.
     */
    virtual NumVector get_velocity() const = 0;

    /*!
     * \brief Get current acceleration vector.
     * \return acceleration vector.
     */
    virtual NumVector get_acceleration() const = 0;

    /*!
     * \brief Get current geodetic coordinates.
     * \param[out] lat current latitude.
     * \param[out] lon current longitude.
     * \param[out] alt current altitude.
     */
    NumVector get_geodetic() const
    {
        return calculate_geodetic(this->get_position());
    }

    /*!
     * \brief Get current spherical coordinates.
     * \return vector of spherical coordinates (latitude, longitude, height).
     */
    NumVector get_spherical() const
    {
        return calculate_spherical(this->get_position());
    }

protected:
    NumVector calculate_geodetic(const NumVector & position) const
    {
        return WrapperWMM::instance().cartesian_to_geodetic(position);
    }

    /*!
     * \brief Calculate spherical coordinates from ECEF.
     * \param position ECEF position vector.
     * \return vector of spherical coordinates (latitude, longitude, height).
     */
    NumVector calculate_spherical(const NumVector & position) const
    {
        NumVector sph(3);
        sph[2] = position.norm();
        sph[0] = std::asin(position[2] / sph[2]);
        sph[1] = std::atan2(position[1], position[0]);
        return sph;
    }

    /*!
     * \brief Accumulate filter input.
     */
    void accumulate(const FilterInput &) override {}

    /*!
     * \brief Initialize filter.
     */
    void initialize(const FilterInput &z) override
    {
        Filter::initialize(z);
        start_geo = z.geo;
        start_ecef = z.pos;
    }

    NumVector start_geo;
    NumVector start_ecef;

    const double ms_to_knots = 1.94384;

    QualityControl vx_ctrl; //!< x-axis bias controller.
    QualityControl vy_ctrl; //!< y-axis bias controller.
    QualityControl vz_ctrl; //!< z-axis bias controller.

private:
    /*!
     * \brief Reset implemented part.
     */
    void reset_this()
    {
        vx_ctrl.reset();
        vy_ctrl.reset();
        vz_ctrl.reset();
    }
};

#endif // POSITIONFILTER_H
