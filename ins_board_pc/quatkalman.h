#ifndef QUATKALMAN_H
#define QUATKALMAN_H

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

namespace ublas = boost::numeric::ublas;
typedef ublas::matrix<double> NumMatrix;
typedef ublas::vector<double> NumVector;

class QuaternionKalman
{
public:
    QuaternionKalman(const NumVector & initial_state,
                     const NumMatrix & initial_errcov_matrix,
                     const NumMatrix & process_noise_covar,
                     const NumMatrix & observation_noise_covar);

    void update(const NumVector & w,
                const NumVector & a,
                const NumVector & m,
                double elapsed_sec);

    NumVector get_state();
    void reset();

private:
    NumVector state;
    NumMatrix err_covar;

    const NumVector init_state;
    const NumMatrix init_err_covar;
    const NumMatrix proc_noise;
    const NumMatrix obs_noise;

    NumVector reference_acceleration;
    NumVector reference_magnetic_field;
};

#endif // QUATKALMAN_H
