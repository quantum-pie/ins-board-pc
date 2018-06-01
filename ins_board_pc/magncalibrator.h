/*! @file magncalibrator.h
  */

#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include "eigenaux.h"

#include <vector>
#include <string>

/*!
 * @brief Magnetometer calibration helper class.
 */
class MagnCalibrator
{
public:
    using meas_iter = std::vector<Vector3D>::const_iterator;

    /*!
     * @brief Class constructor.
     */
	MagnCalibrator();

    /*!
     * @brief Reset calibration parameters to default values.
     */
    void reset();

    /*!
     * @brief Save new measurements.
     * @param m new measured magnetic field vector.
     */
    void update(const Vector3D & m);

    /*!
     * @brief Calibrate magnetometer measurement.
     * @param m measured magnetic field vector to calibrate.
     * @return calibrated magnetometer measurement.
     */
    Vector3D calibrate(const Vector3D & m) const;

    /*!
     * @brief Save calibration parameters to file.
     */
    void save() const;

    /*!
     * @brief Fit calibration parameters to accumulated measurements.
     */
    void fit();

    /*!
     * @brief Fit calibration parameters to accumulated measurements (simple version).
     */
    void fit_simple();

    /*!
     * @brief Get current magnetometer bias.
     * @return magnetometer bias vector.
     */
    Vector3D get_bias() const;

    /*!
     * @brief Get current magnetometer calibration matrix.
     * @return magnetometer calibration matrix.
     */
    Matrix3D get_scale() const;

    meas_iter meas_begin() const;
    meas_iter meas_end() const;
    void clear_meas();

    MagnCalibrator(MagnCalibrator const&) = delete;
    MagnCalibrator& operator =(MagnCalibrator const&) = delete;

private:
    //! Helper matrix alias.
    using C_type = StaticMatrix<6, 6>;

    /*!
     * @brief Load calibration parameters from file.
     */
    void load();

    /*!
     * @brief Create helper matrix.
     * @return helper C matrix.
     */
    static C_type initialize_cmatrix();

    static const C_type C;                                          //!< Helper matrix.

    Vector3D bias;                 				                    //!< Magnetometer bias.
    Matrix3D scale;                				                    //!< Magnetometer calibration matrix.

    static constexpr std::size_t buf_size { 5000 };					//!< Default measurements buffer size.
    static constexpr std::size_t value_size { sizeof(double) };     //!< Size of measured values.
    std::vector<Vector3D> meas;    									//!< Vector of accumulated magnetometer measurements.
};

#endif // CALIBRATOR_H
