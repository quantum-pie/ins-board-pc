#include "quatcomplement.h"

#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/assignment.hpp>

#include <QtMath>

#include <QDebug>

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

    NumMatrix accel_rotator = quaternion_to_dcm(qacc);
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

    NumVector q = quat_multiply(qacc, qmag);

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

NumMatrix QuatComplement::create_quat_bias_mtx(double dt_2)
{
    double q_s = x[0];
    double q_x = x[1];
    double q_y = x[2];
    double q_z = x[3];

    NumMatrix K(4, 3);
    K <<= q_x,  q_y,  q_z,
           -q_s,  q_z, -q_y,
           -q_z, -q_s,  q_x,
            q_y, -q_x, -q_s;

    K *= dt_2;

    return K;
}

NumMatrix QuatComplement::quaternion_to_dcm(const NumVector & quaternion)
{
    double qs = quaternion[0];
    double qx = quaternion[1];
    double qy = quaternion[2];
    double qz = quaternion[3];

    NumMatrix DCM(3, 3);

    double qss = qs * qs;
    double qxx = qx * qx;
    double qyy = qy * qy;
    double qzz = qz * qz;
    double qsx = qs * qx;
    double qsy = qs * qy;
    double qsz = qs * qz;
    double qxy = qx * qy;
    double qxz = qx * qz;
    double qyz = qy * qz;

    DCM(0, 0) = qss + qxx - qyy - qzz;
    DCM(0, 1) = 2 * (qxy + qsz);
    DCM(0, 2) = 2 * (qxz - qsy);
    DCM(1, 0) = 2 * (qxy - qsz);
    DCM(1, 1) = qss - qxx + qyy - qzz;
    DCM(1, 2) = 2 * (qyz + qsx);
    DCM(2, 0) = 2 * (qxz + qsy);
    DCM(2, 1) = 2 * (qyz - qsx);
    DCM(2, 2) = qss - qxx - qyy + qzz;

    return DCM;
}

NumVector QuatComplement::quat_multiply(const NumVector & p, const NumVector & q)
{
    NumVector res(4);

    double p0 = p[0];
    double p1 = p[1];
    double p2 = p[2];
    double p3 = p[3];

    double q0 = q[0];
    double q1 = q[1];
    double q2 = q[2];
    double q3 = q[3];

    res[0] = p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3;
    res[1] = p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2;
    res[2] = p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1;
    res[3] = p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0;

    return res;
}

void QuatComplement::normalize_state()
{
    double qs = x[0];
    double qx = x[1];
    double qy = x[2];
    double qz = x[3];

    double quat_norm = qSqrt(qs * qs + qx * qx + qy * qy + qz * qz);
    x[0] /= quat_norm;
    x[1] /= quat_norm;
    x[2] /= quat_norm;
    x[3] /= quat_norm;
}

void QuatComplement::debug_vector(const NumVector & vec, QString name)
{
    QDebug deb = qDebug();
    deb << name + ":" << endl;
    for(size_t i = 0; i < vec.size(); ++i)
    {
        deb << vec[i];
    }
    deb << endl;
}

void QuatComplement::debug_matrix(const NumMatrix & mtx, QString name)
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

NumVector QuatComplement::get_state()
{
    return x;
}

NumVector QuatComplement::get_orientation_quaternion()
{
    return ublas::vector_range<NumVector>(x, ublas::range(0, 4));
}

NumVector QuatComplement::get_gyro_bias()
{
    return ublas::vector_range<NumVector>(x, ublas::range(4, 7));
}

void QuatComplement::get_rpy(double & roll, double & pitch, double & yaw)
{
    /*! ZXY rotation sequence implied.
     * Explanation:
     * Conventional aerospace rotation sequence is ZYX,
     * but since our coordinate system has Y axis aligned with fuselage,
     * we need to switch rotation order of X and Y.
    */

    double qs = x[0];
    double qx = x[1];
    double qy = x[2];
    double qz = x[3];

    double test = qy * qz + qs * qx;

/*
    if (test > 0.499)
    {
        // singularity at north pole
        yaw = - 2 * qAtan2(qz, qs);
        pitch = M_PI / 2;
        roll = 0;
        return;
    }

    if (test < -0.499)
    {
        // singularity at south pole
        yaw = 2 * qAtan2(qz, qs);
        pitch = - M_PI / 2;
        roll = 0;
        return;
    }
*/

    double qss = qs * qs;
    double qxx = qx * qx;
    double qyy = qy * qy;
    double qzz = qz * qz;

    yaw = -qAtan2(2 * qs * qz - 2 * qx * qy, qss - qxx + qyy - qzz);
    pitch = qAsin(2 * test);
    roll = qAtan2(2 * qs * qy - 2 * qx * qz, qss - qxx - qyy + qzz);
}
