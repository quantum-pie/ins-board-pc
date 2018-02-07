#ifndef UBLASTYPES_H
#define UBLASTYPES_H

#include <QString>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

namespace ublas = boost::numeric::ublas;
using NumMatrix = ublas::matrix<double>;
using NumVector = ublas::vector<double>;
using ZeroMatrix = ublas::zero_matrix<double>;
using ZeroVector = ublas::zero_vector<double>;
using IdentityMatrix = ublas::identity_matrix<double>;

namespace uaux
{

void debug_vector(const NumVector & vec, QString name);

void debug_matrix(const NumMatrix & mtx, QString name);

bool invert_matrix(const NumMatrix & mtx, NumMatrix & inv);

}

#endif // UBLASTYPES_H
