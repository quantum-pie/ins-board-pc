#ifndef UBLASTYPES_H
#define UBLASTYPES_H

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

namespace ublas = boost::numeric::ublas;
using NumMatrix = ublas::matrix<double>;
using NumVector = ublas::vector<double>;
using ZeroMatrix = ublas::zero_matrix<double>;
using ZeroVector = ublas::zero_vector<double>;
using IdentityMatrix = ublas::identity_matrix<double>;

#endif // UBLASTYPES_H
