#ifndef UBLASTYPES_H
#define UBLASTYPES_H

#include <QString>

#include <Eigen/Core>

using NumMatrix = Eigen::MatrixXd;
using NumVector = Eigen::VectorXd;

namespace eaux
{

void debug_vector(const NumVector & vec, QString name);
void debug_matrix(const NumMatrix & mtx, QString name);

}

#endif // UBLASTYPES_H
