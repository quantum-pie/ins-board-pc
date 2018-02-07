#include "quaternions.h"

#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/assignment.hpp>

#include <QtMath>

namespace qutils
{

void quat_to_rpy(const NumVector & quaternion, double & roll, double & pitch, double & yaw)
{
    /*! ZXY rotation sequence implied.
     * Explanation:
     * Conventional aerospace rotation sequence is ZYX,
     * but since our coordinate system has Y axis aligned with fuselage,
     * we need to switch rotation order of X and Y.
    */

    double qs = quaternion[0];
    double qx = quaternion[1];
    double qy = quaternion[2];
    double qz = quaternion[3];

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

NumMatrix quaternion_to_dcm(const NumVector & quaternion)
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

NumVector quat_multiply(const NumVector & p, const NumVector & q)
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

NumVector quat_normalize(const NumVector & quaternion)
{
    double qs = quaternion[0];
    double qx = quaternion[1];
    double qy = quaternion[2];
    double qz = quaternion[3];

    double quat_norm = qSqrt(qs * qs + qx * qx + qy * qy + qz * qz);

    NumVector res(4);

    res[0] = quaternion[0] / quat_norm;
    res[1] = quaternion[1] / quat_norm;
    res[2] = quaternion[2] / quat_norm;
    res[3] = quaternion[3] / quat_norm;

    return res;
}

NumMatrix quat_delta_mtx(const NumVector & quaternion, double dt_2)
{
    double q_s = quaternion[0];
    double q_x = quaternion[1];
    double q_y = quaternion[2];
    double q_z = quaternion[3];

    NumMatrix K(4, 3);
    K <<= q_x,  q_y,  q_z,
           -q_s,  q_z, -q_y,
           -q_z, -q_s,  q_x,
            q_y, -q_x, -q_s;

    K *= dt_2;

    return K;
}

NumMatrix skew_symmetric(const NumVector & v)
{
    NumMatrix V(4, 4);
    V <<=    0,    -v[0], -v[1], -v[2],
             v[0],  0,     v[2], -v[1],
             v[1], -v[2],  0,     v[0],
             v[2],  v[1], -v[0],  0;

    return V;
}

NumMatrix ddcm_dqs(const NumVector & quaternion)
{
    double qs = quaternion[0];
    double qx = quaternion[1];
    double qy = quaternion[2];
    double qz = quaternion[3];

    NumMatrix RES(3, 3);

    RES <<= qs, qz, -qy,
            -qz, qs, qx,
             qy, -qx, qs;

    RES *= 2;

    return RES;
}

NumMatrix ddcm_dqx(const NumVector & quaternion)
{
    double qs = quaternion[0];
    double qx = quaternion[1];
    double qy = quaternion[2];
    double qz = quaternion[3];

    NumMatrix RES(3, 3);

    RES <<= qx, qy, qz,
            qy, -qx, qs,
            qz, -qs, -qx;

    RES *= 2;

    return RES;
}

NumMatrix ddcm_dqy(const NumVector & quaternion)
{
    double qs = quaternion[0];
    double qx = quaternion[1];
    double qy = quaternion[2];
    double qz = quaternion[3];

    NumMatrix RES(3, 3);

    RES <<= -qy, qx, -qs,
            qx, qy, qz,
            qs, qz, -qy;

    RES *= 2;

    return RES;
}

NumMatrix ddcm_dqz(const NumVector & quaternion)
{
    double qs = quaternion[0];
    double qx = quaternion[1];
    double qy = quaternion[2];
    double qz = quaternion[3];

    NumMatrix RES(3, 3);

    RES <<= -qz, qs, qx,
            -qs, -qz, qy,
            qx, qy, qz;

    RES *= 2;

    return RES;
}

NumVector acceleration_quat(const NumVector & a)
{
    NumVector a_norm = a / norm_2(a);

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

    return qacc;
}

NumVector magnetometer_quat(const NumVector & l)
{
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

    return qmag;
}

NumVector declinator_quat(double declination)
{
    NumVector qdecl(4);

    // rotate magnetic field by declination degrees around Z axis counterclockwise
    qdecl[0] = qCos(declination / 2);
    qdecl[1] = 0;
    qdecl[2] = 0;
    qdecl[3] = qSin(declination / 2);

    return qdecl;
}

}
