/*! \file calibrator.h
  */

#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include "eigenaux.h"

#include <QVector3D>

/*!
 * \brief Magnetemoter calibration helper class.
 */
class Calibrator
{
public:
    /*!
     * \brief Constructor.
     */
    Calibrator();

    /*!
     * \brief Reset calibration parameters to default values.
     */
    void reset();

    /*!
     * \brief Save new measurements.
     * \param m new measured magnetic field vector.
     */
    void update(const NumVector & m);

    /*!
     * \brief Calibrate magnetometer measurement in place.
     * \param m measured magnetic field vector to calibrate.
     */
    void calibrate(NumVector & m) const;

    /*!
     * \brief Calibrate magnetometer measurement.
     * \param m measured magnetic field vector to calibrate.
     * \return calibrated magnetometer measurement.
     */
    NumVector calibrate(const NumVector & m) const;

    /*!
     * \brief Calibrate magnetometer measurement.
     * \param vec measured magnetic field vector to calibrate.
     * \return calibrated magnetometer measurement.
     */
    QVector3D calibrate(const QVector3D & vec) const;

    /*!
     * \brief Save calibration parameters to file.
     */
    void save() const;

    /*!
     * \brief Fit calibration parameters to accumulated measurements.
     */
    void fit();

    /*!
     * \brief Fit calibration parameters to accumulated measurements (simple version).
     */
    void fit_simple();

    /*!
     * \brief Get current magnetometer bias.
     * \return magnetometer bias vector.
     */
    NumVector get_bias() const;

    /*!
     * \brief Get current magnetometer calibration matrix.
     * \return magnetometer calibration matrix.
     */
    NumMatrix get_scale() const;

private:
    /*!
     * \brief Load calibration parameters from file.
     */
    void load();

    NumVector bias;                 //!< Magnetometer bias.
    NumMatrix scale;                //!< Magnetometer calibration matrix.

    std::vector<NumVector> meas;    //!< Vector of accumulated magnetometer measurements.
};

#endif // CALIBRATOR_H
