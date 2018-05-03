/*! \file filter.h
  */

#ifndef FILTER_H
#define FILTER_H

#include "eigenaux.h"

#include <QDate>

/*!
 * \brief Abstract filter class.
 *
 * This class represents most basic filter entity.
 */
class Filter
{
public:
    /*!
     * \brief Kalman filter input type.
     */
    struct FilterInput
    {
        NumVector w;    //!< angular rate vector in dps.
        NumVector a;    //!< accelerometer readings vector in g.
        NumVector m;    //!< normalized magnetometer readings vector.
        QDate day;      //!< current date.
        NumVector geo;  //!< geodetic coordinates vector ( latitude and longitude in deg, altitude above ellipsoid in m).
        NumVector pos;  //!< position vector in cartesian coordinates in m.
        NumVector v;    //!< velocity vector in cartesian coordinates in m/s.
        double dt;      //!< elapsed time in s.
        bool gps_fresh; //!< GPS data is fresh.
    };

    /*!
     * \brief Constructor.
     */
    Filter()
    {
        reset_this();
    }

    /*!
     * \brief Destructor.
     */
    virtual ~Filter() {}

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
     */
    virtual void initialize(const FilterInput &)
    {
        initialized = true;
    }

    NumVector x;        //!< Filter state vector.

private:
    bool initialized;   //!< Filter initialization state.

    /*!
     * \brief Reset implemented part.
     */
    void reset_this()
    {
        initialized = false;
    }
};

#endif // FILTER_H
