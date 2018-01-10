#include "quatkalman.h"

#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/numeric/ublas/assignment.hpp>

#include <iostream>
#include <boost/numeric/ublas/io.hpp>

#include <QtMath>

QuaternionKalman::QuaternionKalman(const NumVector &initial_state,
                                   const NumMatrix &initial_errcov_matrix,
                                   const NumMatrix &process_noise_covar,
                                   const NumMatrix &observation_noise_covar)
    : init_state(initial_state),
      init_err_covar(initial_errcov_matrix),
      proc_noise(process_noise_covar),
      obs_noise(observation_noise_covar)
{
    reset();
}

void QuaternionKalman::reset()
{
    state = init_state;
    err_covar = init_err_covar;
}

void QuaternionKalman::update(const NumVector &w,
                         const NumVector &a,
                         const NumVector &m, double dt)
{
    double w_len = norm_2(w);

    NumMatrix V(4, 4);
    V <<=    0,    -w[0], -w[1],  w[2],
             w[0],  0,     w[2], -w[1],
             w[1], -w[2],  0,     w[0],
             w[2],  w[1], -w[0],  0;

    double q_s = state[0];
    double q_x = state[1];
    double q_y = state[2];
    double q_z = state[3];

    NumMatrix q_b(4, 3);
    q_b <<= q_x,  q_y,  q_z,
           -q_s,  q_z, -q_y,
           -q_z, -q_s,  q_x,
            q_y, -q_x, -q_s;

    NumMatrix F = ublas::identity_matrix<double>(7) * qCos(dt * w_len / 2);
    F(4, 4) = 1;
    F(5, 5) = 1;
    F(6, 6) = 1;

    NumMatrix Fm(7, 7);
    Fm <<= V, q_b,
            ublas::zero_matrix<double>(3, 7);

    F += Fm * qSin(dt * w_len / 2) / w_len;

    state = prod(F, state);

    NumMatrix tmp = prod(F, err_covar);
    err_covar = prod(tmp, trans(F)) + proc_noise;

    q_s = state[0];
    q_x = state[1];
    q_y = state[2];
    q_z = state[3];
}

NumVector QuaternionKalman::get_state()
{
    return state;
}
