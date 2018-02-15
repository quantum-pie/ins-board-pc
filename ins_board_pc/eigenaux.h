#ifndef UBLASTYPES_H
#define UBLASTYPES_H

#include <QString>

#include <Eigen/Core>

using NumMatrix = Eigen::MatrixXd;
using NumVector = Eigen::VectorXd;


//using ZeroMatrix = ublas::zero_matrix<double>;
//using ZeroVector = ublas::zero_vector<double>;
//using IdentityMatrix = ublas::identity_matrix<double>;

namespace eaux
{

void debug_vector(const NumVector & vec, QString name);

void debug_matrix(const NumMatrix & mtx, QString name);

//bool invert_matrix(const NumMatrix & mtx, NumMatrix & inv);

}

#endif // UBLASTYPES_H
