#include "ublasaux.h"
#include <boost/numeric/ublas/lu.hpp>

#include <QDebug>

namespace uaux
{

void debug_vector(const NumVector & vec, QString name)
{
    QDebug deb = qDebug();
    deb << name + ":" << endl;
    for(size_t i = 0; i < vec.size(); ++i)
    {
        deb << vec[i];
    }
    deb << endl;
}

void debug_matrix(const NumMatrix & mtx, QString name)
{
    QDebug deb = qDebug();
    deb << name + ":" << endl;
    for(size_t i = 0; i < mtx.size1(); ++i)
    {
        for(size_t j = 0; j < mtx.size2(); ++j)
        {
            deb << mtx(i, j);
        }
        deb << endl;
    }
}

bool invert_matrix(const NumMatrix & mtx, NumMatrix & inv)
{
    typedef ublas::permutation_matrix<std::size_t> pmatrix;

    NumMatrix A(mtx);

    // create a permutation matrix for the LU-factorization
    pmatrix pm(A.size1());

    // perform LU-factorization
    int res = lu_factorize(A, pm);
    if (res != 0)
        return false;

    // create identity matrix of "inverse"
    inv.assign(IdentityMatrix(A.size1()));

    // backsubstitute to get the inverse
    lu_substitute(A, pm, inv);

    return true;
}

}

