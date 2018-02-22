/*! \file abstractpositionfilter.h
  */

#ifndef ABSTRACTPOSITIONFILTER_H
#define ABSTRACTPOSITIONFILTER_H

#include "abstractfilter.h"
#include "wmmwrapper.h"
#include "quaternions.h"

/*!
 * \brief Abstract point position filter.
 *
 * Base class for all filters capable of estimating point position.
 */
class AbstractPositionFilter : public virtual AbstractFilter
{
public:
    /*!
     * \brief Constructor.
     */
    AbstractPositionFilter() : AbstractFilter() {}

    /*!
     * \brief Destructor.
     */
    ~AbstractPositionFilter() override {}

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
        NumMatrix Cel = WrapperWMM::instance().geodetic_to_dcm(start_geo[0], start_geo[1]);
        return Cel * (pos - start_ecef);
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
    void get_geodetic(double & lat, double & lon, double & alt) const
    {
        calculate_geodetic(this->get_position(), lat, lon, alt);
    }

protected:
    void calculate_geodetic(const NumVector & position,
                            double & lat, double & lon, double & alt) const
    {
        WrapperWMM::instance().cartesian_to_geodetic(position, lat, lon, alt);
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
        AbstractFilter::initialize(z);
        start_geo = z.geo;
        start_ecef = z.pos;
    }

    NumVector start_geo;
    NumVector start_ecef;
};

#endif // ABSTRACTPOSITIONFILTER_H
