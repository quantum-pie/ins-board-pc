/*! \file abstractfilter.h
  */

#ifndef ABSTRACTFILTER_H
#define ABSTRACTFILTER_H

#include "eigenaux.h"

#include <QDate>

/*!
 * \brief Abstract filter class.
 *
 * This class represents most basic filter entity.
 */
class AbstractFilter
{
public:
    //! Filter input type.
    struct FilterInput
    {
        NumVector w;    //!< angular rate vector in \f$ \big[ dps \big] \f$.
        NumVector a;    //!< accelerometer readings vector in \f$ \big[ g \big] \f$.
        NumVector m;    //!< normalized magnetometer readings vector.
        QDate day;      //!< current date.
        NumVector geo;  //!< geodetic coordinates vector ( latitude and longitude in \f$ \big[ deg \big] \f$, altitude above ellipsoid in \f$ \big[ m \big] \f$).
        NumVector pos;  //!< position vector in cartesian coordinates in \f$ \big[ m \big] \f$.
        NumVector v;    //!< velocity vector in cartesian coordinates in \f$ \big[ \frac{m}{s} \big] \f$.
        double dt;      //!< elapsed time in \f$ \big[ s \big] \f$.
    };

    /*!
     * \brief Constructor.
     */
    AbstractFilter()
    {
        reset_this();
    }

    /*!
     * \brief Destructor.
     */
    virtual ~AbstractFilter() {}

    /*!
     * \brief Filter step.
     * \param z filter input reference.
     */
    virtual void step(const FilterInput & z) = 0;

    /*!
     * \brief Get filter state vector.
     * \return filter state vector.
     */
    NumVector get_state() const
    {
        return x;
    }

    /*!
     * \brief Reset filter.
     */
    virtual void reset()
    {
        reset_this();
    }

    /*!
     * \brief check filter initialization state.
     * \return true of filter is initialized.
     */
    bool is_initialized() const
    {
        return initialized;
    }

protected:
    /*!
     * \brief Update filter state.
     * \param z filter input reference.
     */
    virtual void update(const FilterInput & z) = 0;

    /*!
     * \brief Accumulate filter input.
     * \param z filter input reference.
     */
    virtual void accumulate(const FilterInput & z) = 0;

    /*!
     * \brief Initialize filter.
     * \param filter input reference.
     */
    virtual void initialize(const FilterInput &)
    {
        initialized = true;
    }

    NumVector x; //!< Filter state vector.

private:
    bool initialized; //!< Filter initialization state.

    /*!
     * \brief Reset implemented part.
     */
    void reset_this()
    {
        initialized = false;
    }
};

#endif // ABSTRACTFILTER_H
