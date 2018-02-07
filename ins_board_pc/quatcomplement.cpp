#include "quatcomplement.h"

#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/assignment.hpp>

#include <QtMath>

QuatComplement::QuatComplement(const FilterParams & params)
    : params(params)
{
    reset();
    x = NumVector(state_size);
}

void QuatComplement::accumulate(const ComplementInput & z)
{
    accum.w += z.w;
    accum.a += z.a;
    accum.m += z.m;
    accum.dt = z.dt;

    accum_size++;
}

void QuatComplement::initialize()
{
    NumVector gyro_bias = accum.w / accum_size;
    accum.a /= accum_size;
    accum.m /= accum_size;

    NumVector a_norm = accum.a / norm_2(accum.a);

    double ax = a_norm[0];
    double ay = a_norm[1];
    double az = a_norm[2];

    NumVector qacc(4);
    if(az >= 0)
    {
        qacc[0] = qSqrt( (az + 1) / 2);
        qacc[1] = - ay / qSqrt(2 * (az + 1));
        qacc[2] = ax / qSqrt(2 * (az + 1));
        qacc[3] = 0;
    }
    else
    {
        qacc[0] = - ay / qSqrt(2 * (1 - az));
        qacc[1] = qSqrt( (1 - az) / 2);
        qacc[2] = 0;
        qacc[3] = ax / qSqrt(2 * (1 - az));
    }

    NumMatrix accel_rotator = qutils::quaternion_to_dcm(qacc);
    NumVector l = prod(accel_rotator, accum.m);

    double lx = l[0];
    double ly = l[1];

    NumVector qmag(4);
    double G = lx * lx + ly * ly;
    double G_sqrt = qSqrt(G);

    if(ly >= 0)
    {
        qmag[0] = qSqrt(G + ly * G_sqrt) / qSqrt(2 * G);
        qmag[1] = 0;
        qmag[2] = 0;
        qmag[3] = - lx / qSqrt(2 * (G + ly * G_sqrt));
    }
    else
    {
        qmag[0] = - lx / qSqrt(2 * (G - ly * G_sqrt));
        qmag[1] = 0;
        qmag[2] = 0;
        qmag[3] = qSqrt(G - ly * G_sqrt) / qSqrt(2 * G);
    }

    NumVector q = qutils::quat_multiply(qacc, qmag);

    // from lb to bl quaternion
    x[0] = q[0];
    x[1] = -q[1];
    x[2] = -q[2];
    x[3] = -q[3];

    x[4] = gyro_bias[0];
    x[5] = gyro_bias[1];
    x[6] = gyro_bias[2];
}

void QuatComplement::reset()
{
    initialized = false;
    accum_size = 0;

    accum.w = ZeroVector(3);
    accum.a = ZeroVector(3);
    accum.m = ZeroVector(3);
    accum.dt = 0;
}

bool QuatComplement::is_initialized()
{
    return initialized;
}

void QuatComplement::step(const ComplementInput & z)
{
    if(initialized)
    {
        update(z);
    }
    else
    {
        accumulate(z);
        if(accum_size == accum_capacity)
        {
            initialize();
            initialized = true;
        }
    }
}

void QuatComplement::update(const ComplementInput & z)
{
    // TODO
}

void QuatComplement::normalize_state()
{
    ublas::project(x, ublas::range(0, 4)) = qutils::quat_normalize(get_orientation_quaternion());
}

NumVector QuatComplement::get_state()
{
    return x;
}

NumVector QuatComplement::get_orientation_quaternion()
{
    return ublas::project(x, ublas::range(0, 4));
}

NumVector QuatComplement::get_gyro_bias()
{
    return ublas::project(x, ublas::range(4, 7));
}

void QuatComplement::get_rpy(double & roll, double & pitch, double & yaw)
{
    qutils::quat_to_rpy(get_orientation_quaternion(), roll, pitch, yaw);
}
