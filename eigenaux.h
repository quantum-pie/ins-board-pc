/*! \file eigenaux.h
  */

#ifndef EIGENAUX_H
#define EIGENAUX_H

#include <string>
#include <iostream>

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
 * @brief Debug vector.
 * @tparam V vector type.
 * @param vec vector.
 * @param name vector name.
 */
template<typename V>
void debug_vector(const V & vec, const std::string & name)
{
    std::cout << name + ":" << std::endl;
    for(int i = 0; i < vec.size(); ++i)
    {
        std::cout << vec[i] << ' ';
    }
    std::cout << std::endl;
}

/*!
 * @brief Debug matrix.
 * @tparam M matrix type.
 * @param mtx matrix.
 * @param name matrix name.
 */
template<typename M>
void debug_matrix(const M & mtx, const std::string & name)
{
    std::cout << name + ":" << std::endl;
    for(int i = 0; i < mtx.rows(); ++i)
    {
        for(int j = 0; j < mtx.cols(); ++j)
        {
            std::cout << mtx(i, j) << ' ';
        }
        std::cout << std::endl;
    }
}

}

#endif // EIGENAUX_H
