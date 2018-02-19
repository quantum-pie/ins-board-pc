/*! \file abstractpositionfilter.h
  */

#ifndef ABSTRACTPOSITIONFILTER_H
#define ABSTRACTPOSITIONFILTER_H

#include "abstractfilter.h"

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
     * \brief Get current position vector.
     * \return position vector.
     */
    virtual NumVector get_position() const = 0;

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
    virtual void get_geodetic(double & lat, double & lon, double & alt) const = 0;

protected:
    /*!
     * \brief Accumulate filter input.
     */
    void accumulate(const FilterInput &) override {}
};

#endif // ABSTRACTPOSITIONFILTER_H
