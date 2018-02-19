/*! \file eigenaux.h
  */

#ifndef EIGENAUX_H
#define EIGENAUX_H

#include <QString>

#include <Eigen/Core>

//! Numeric matrix alias.
using NumMatrix = Eigen::MatrixXd;

//! Numeric vector alias.
using NumVector = Eigen::VectorXd;

namespace eaux
{

/*!
 * \brief Display vector via qDebug().
 * \param vec vector to display.
 * \param name vector label.
 */
void debug_vector(const NumVector & vec, QString name);

/*!
 * \brief Display matrix via qDebug().
 * \param mtx matrix to display.
 * \param name matrix label.
 */
void debug_matrix(const NumMatrix & mtx, QString name);

}

#endif // EIGENAUX_H
