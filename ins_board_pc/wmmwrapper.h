/*! \file wmmwrapper.h
  */

#ifndef WMMWRAPPER_H
#define WMMWRAPPER_H

extern "C" {
    #include "wmm/GeomagnetismHeader.h"
}

#include "eigenaux.h"

#include <QDate>

/*!
 * \brief Wrapper singleton class for World Magnetic Model.
 */
class WrapperWMM
{
public:
    /*!
     * \brief Get instance of WMM wrapper.
     * \return return WMM wrapper instance reference.
     */
    static WrapperWMM& instance()
    {
         static WrapperWMM wmm;
         return wmm;
    }

    /*!
     * \brief No copy contructor for singleton.
     */
    WrapperWMM(WrapperWMM const&) = delete;

    /*!
     * \brief No assignment operator for singleton.
     */
    WrapperWMM& operator =(WrapperWMM const&) = delete;

    /*!
     * \brief Measure magnetic field parameters at a given time and position.
     * \param lat geodetic latitude.
     * \param lon geodetic longitude.
     * \param alt geodetic altitude above ellipsoid.
     * \param day current date.
     * \param[out] declination magnetic field declination angle.
     * \param[out] inclination magnetic field inclination angle.
     * \param[out] magnitude magnetic field magnitude.
     */
    void measure(double lat, double lon, double alt, QDate day, double & declination, double & inclination, double & magnitude);

    /*!
     * \brief Get WGS-84 ellipsoid a-parameter.
     * \return elipsoid a-parameter.
     */
    double ellip_a() const;

    /*!
     * \brief Get WGS-84 ellipsoid epsilon squared parameter.
     * \return elipsoid epsilon squared parameter.
     */
    double ellip_epssq() const;

    /*!
     * \brief Get WGS-84 mean Earth radius.
     * \return mean Earth radius.
     */
    double earth_rad() const;

    /*!
     * \brief Get WGS-84 ellipsoid f-parameter.
     * \return elipsoid f-parameter.
     */
    double ellip_f() const;

    /*!
     * \brief Convert cartesian to geodetic coordinates.
     * \param pos cartresian position vector.
     * \param[out] lat geodetic latitude.
     * \param[out] lon geodetic longitude.
     * \param[out] alt geodetic altitude above ellipsoid.
     */
    void cartesian_to_geodetic(const NumVector & pos, double & lat, double & lon, double & alt) const;

    /*!
     * \brief Convert geodetic coordinates to direction cosine matrix.
     * \param lat geodetic latitude.
     * \param lon geodetic longitude.
     * \return direction cosine matrix.
     */
    NumMatrix geodetic_to_dcm(double lat, double lon) const;

    /*!
     * \brief Calculate model normalized magnetic field vector at a given time and position.
     * \param lat geodetic latitude.
     * \param lon geodetic longitude.
     * \param alt geodetic altitude above ellipsoid.
     * \param day current date.
     * \return expected normalized magnetic field vector.
     */
    NumVector expected_mag(double lat, double lon, double alt, QDate day);

    /*!
     * \brief Calculate model magnetic field vector magnitude at a given time and position.
     * \param lat geodetic latitude.
     * \param lon geodetic longitude.
     * \param alt geodetic altitude above ellipsoid.
     * \param day current date.
     * \return expected magnetic field vector magnitude.
     */
    double expected_mag_magnitude(double lat, double lon, double alt, QDate day);

    /*!
     * \brief Calculate model gravity acceleration vector at a given position.
     * \param lat geodetic latitude.
     * \param alt geodetic altitude above ellipsoid.
     * \return expected gravity acceleration vector.
     */
    double expected_gravity_accel(double lat, double alt) const;

    /*!
     * \brief Calculate derivative of direction cosine matrix with respect to latitude given position.
     * \param lat geodetic latitude.
     * \param lon geodetic longitude.
     * \return derivative of DCM with respect to latitude.
     */
    NumMatrix dcm_lat_partial(double lat, double lon) const;

    /*!
     * \brief Calculate derivative of direction cosine matrix with respect to longitude given position.
     * \param lat geodetic latitude.
     * \param lon geodetic longitude.
     * \return derivative of DCM with respect to longitude.
     */
    NumMatrix dcm_lon_partial(double lat, double lon) const;

    /*!
     * \brief Calculate derivative of geodetic coordinates with respect to cartesian ones.
     * \param lat geodetic latitude.
     * \param lon geodetic longitude.
     * \param alt geodetic altitude above ellipsoid.
     * \return derivative of geodetic coordinates with respect to cartesian ones.
     */
    NumMatrix dgeo_dpos(double lat, double lon, double alt) const;

private:
    /*!
     * \brief Constructor.
     * \param params filter parameters.
     */
    WrapperWMM();

    MAGtype_Ellipsoid ellip;                        //!< Ellipsoid parameters instance.
    MAGtype_Geoid geoid;                            //!< Geoid parameters instance.
    MAGtype_MagneticModel ** magnetic_models;       //!< Array of pointers to the magnetic models.
    MAGtype_MagneticModel *  timed_magnetic_model;  //!< Pointer to the current magnetic model.

    char err_msg[255];                              //!< Error message buffer.
};

#endif // WMMWRAPPER_H
