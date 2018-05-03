/*
 * quatrenion.cpp
 *
 *      Author: bigaw
 */

#include "quaternion.h"

const Quaternion Quaternion::identity {1.0, 0, 0, 0};

Quaternion Quaternion::x_rotator(double radians)
{
	return {std::cos(radians / 2), std::sin(radians / 2), 0.0, 0.0};
}

Quaternion Quaternion::y_rotator(double radians)
{
	return {std::cos(radians / 2), 0.0, std::sin(radians / 2), 0.0};
}

Quaternion Quaternion::z_rotator(double radians)
{
    return {std::cos(radians / 2), 0.0, 0.0, std::sin(radians / 2)};
}

Quaternion Quaternion::acceleration_quat(const Vector3D & a)
{
	Vector3D a_norm = a / a.norm();

    double ax = a_norm[0];
    double ay = a_norm[1];
    double az = a_norm[2];

    if(az >= 0)
    {
    	return { std::sqrt( (az + 1) / 2),
    			 -ay / std::sqrt(2 * (az + 1)),
				 ax / std::sqrt(2 * (az + 1)),
    			 0.0 };
    }
    else
    {
    	return { -ay / std::sqrt(2 * (1 - az)),
				 std::sqrt( (1 - az) / 2),
				 0.0,
				 ax / std::sqrt(2 * (1 - az)) };
    }
}

Quaternion Quaternion::magnetometer_quat(const Vector3D & l)
{
    double lx = l[0];
    double ly = l[1];

    double G = lx * lx + ly * ly;
    double G_sqrt = std::sqrt(G);

    if(ly >= 0)
    {
    	return { std::sqrt(G + ly * G_sqrt) / std::sqrt(2 * G),
    			 0.0,
				 0.0,
				 -lx / std::sqrt(2 * (G + ly * G_sqrt)) };
    }
    else
    {
    	return { -lx / std::sqrt(2 * (G - ly * G_sqrt)),
    			 0.0,
				 0.0,
				 std::sqrt(G - ly * G_sqrt) / std::sqrt(2 * G) };
    }
}

Quaternion::skew_type Quaternion::skew_symmetric(const Vector3D & v)
{
	skew_type V;
    V <<     0,    -v[0], -v[1], -v[2],
             v[0],  0,     v[2], -v[1],
             v[1], -v[2],  0,     v[0],
             v[2],  v[1], -v[0],  0;

    return V;
}

Quaternion::Quaternion(double qs, double qx, double qy, double qz)
						: qs{qs}, qx{qx}, qy{qy}, qz{qz}
{}

Quaternion::Quaternion(const Vector3D & v)
						: Quaternion{0.0, v[0], v[1], v[2]}
{}

Quaternion::Quaternion(const vector_form & v)
						: Quaternion{v[0], v[1], v[2], v[3]}
{}

Vector3D Quaternion::rpy() const
{
    /*! ZXY rotation sequence implied.
     * Explanation:
     * Conventional aerospace rotation sequence is ZYX,
     * but since our coordinate system has Y axis aligned with fuselage,
     * we need to switch rotation order of X and Y.
    */
    double test = qy * qz + qs * qx;

    double qss = qs * qs;
    double qxx = qx * qx;
    double qyy = qy * qy;
    double qzz = qz * qz;

    Vector3D res;
    res[0] = std::atan2(2 * qs * qy - 2 * qx * qz, qss - qxx - qyy + qzz);
    res[1] = std::asin(2 * test);
    res[2] = -std::atan2(2 * qs * qz - 2 * qx * qy, qss - qxx + qyy - qzz);

    return res;
}

Matrix3D Quaternion::dcm() const
{
	Matrix3D DCM;

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
    DCM(0, 1) = 2 * (qxy - qsz);
    DCM(0, 2) = 2 * (qxz + qsy);
    DCM(1, 0) = 2 * (qxy + qsz);
    DCM(1, 1) = qss - qxx + qyy - qzz;
    DCM(1, 2) = 2 * (qyz - qsx);
    DCM(2, 0) = 2 * (qxz - qsy);
    DCM(2, 1) = 2 * (qyz + qsx);
    DCM(2, 2) = qss - qxx - qyy + qzz;

    return DCM;
}

Matrix3D Quaternion::dcm_tr() const
{
	Matrix3D DCM;

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

Quaternion::operator vector_form() const
{
	vector_form res;
	res << qs, qx, qy, qz;
	return res;
}

Quaternion::delta_type Quaternion::delta_mtx(double dt_2) const
{
	delta_type K;
    K <<    qx,  qy,  qz,
           -qs,  qz, -qy,
           -qz, -qs,  qx,
            qy, -qx, -qs;

    return K * dt_2;
}

double Quaternion::norm() const
{
	return std::sqrt(qs * qs + qx * qx + qy * qy + qz * qz);
}

Quaternion& Quaternion::normalize()
{
    double quat_norm = norm();

    qs /= quat_norm;
    qx /= quat_norm;
    qy /= quat_norm;
    qz /= quat_norm;

    return *this;
}

Quaternion Quaternion::conjugate() const
{
	return {qs, -qx, -qy, -qz};
}

double Quaternion::scalar_part() const
{
	return qs;
}

Vector3D Quaternion::vector_part() const
{
	Vector3D im;
	im << qx, qy, qz;
	return im;
}

bool Quaternion::is_real() const
{
	return qs == 0;
}

bool Quaternion::is_pure_imag() const
{
	return vector_part() == Vector3D::Zero();
}

Matrix3D Quaternion::ddcm_dqs_tr() const
{
	Matrix3D RES;

    RES <<   qs, qz, -qy,
            -qz, qs, qx,
             qy, -qx, qs;

    return RES * 2;
}

Matrix3D Quaternion::ddcm_dqx_tr() const
{
	Matrix3D RES;

    RES <<  qx, qy, qz,
            qy, -qx, qs,
            qz, -qs, -qx;

    return RES * 2;
}

Matrix3D Quaternion::ddcm_dqy_tr() const
{
	Matrix3D RES;

    RES << -qy, qx, -qs,
            qx, qy, qz,
            qs, qz, -qy;

    return RES * 2;
}

Matrix3D Quaternion::ddcm_dqz_tr() const
{
	Matrix3D RES;

    RES << -qz, qs, qx,
            -qs, -qz, qy,
            qx, qy, qz;

    return RES * 2;
}

Quaternion Quaternion::operator*(const Quaternion & p) const
{
    return { qs * p.qs - qx * p.qx - qy * p.qy - qz * p.qz,
    	     qs * p.qx + qx * p.qs + qy * p.qz - qz * p.qy,
			 qs * p.qy - qx * p.qz + qy * p.qs + qz * p.qx,
			 qs * p.qz + qx * p.qy - qy * p.qx + qz * p.qs };
}

Quaternion& Quaternion::operator*=(const Quaternion & p)
{
	return *this = *this * p;
}

Quaternion Quaternion::operator*(double v) const
{
	Quaternion res{*this};
	return res *= v;
}

Quaternion& Quaternion::operator*=(double v)
{
	qs *= v;
	qx *= v;
	qy *= v;
	qz *= v;

	return *this;
}

Quaternion Quaternion::operator+(const Quaternion & p) const
{
	Quaternion res{*this};
	return res += p;
}

Quaternion& Quaternion::operator+=(const Quaternion & p)
{
	qs += p.qs;
	qx += p.qx;
	qy += p.qy;
	qz += p.qz;

	return *this;
}

Quaternion lerp(const Quaternion & q, const Quaternion & p, double alpha)
{
    return (q * (1 - alpha) + p * alpha).normalize();
}

Quaternion slerp(const Quaternion & q, const Quaternion & p, double alpha)
{
    // TODO
	return lerp(q, p, alpha);
}
