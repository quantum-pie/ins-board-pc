/*! \file eigenaux.h
  */

#ifndef EIGENAUX_H
#define EIGENAUX_H

#include <string>

#include <Eigen/Core>

//! Dynamic-sized numeric matrix alias.
using DynamicMatrix = Eigen::MatrixXd;

//! Dynamic-sized numeric vector alias.
using DynamicVector = Eigen::VectorXd;

//! Static-sized numeric matrix alias.
template<std::size_t D1, std::size_t D2>
using StaticMatrix = Eigen::Matrix<double, D1, D2>;

//! Static-sized numeric vector alias.
template<std::size_t D>
using StaticVector = Eigen::Matrix<double, D, 1>;

//! 3D matrix alias.
using Matrix3D = Eigen::Matrix3d;

//! 3D Vector alias.
using Vector3D = Eigen::Vector3d;

/*!
 * @brief This namespace holds implementation of various linear algebra utility functions.
 */
namespace eaux
{

/*!
 * @brief Display vector.
 * @param vec vector to display.
 * @param name vector label.
 */
void debug_vector(const DynamicVector & vec, std::string name);

/*!
 * @brief Display matrix.
 * @param mtx matrix to display.
 * @param name matrix label.
 */
void debug_matrix(const DynamicMatrix & mtx, std::string name);

}

#endif // EIGENAUX_H
