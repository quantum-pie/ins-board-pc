#ifndef QUATERNIONS_H
#define QUATERNIONS_H

#include "eigenaux.h"

namespace qutils
{

void quat_to_rpy(const NumVector & quaternion, double & roll, double & pitch, double & yaw);

NumMatrix quaternion_to_dcm_tr(const NumVector & quaternion);
NumMatrix quaternion_to_dcm(const NumVector & quaternion);
NumVector quat_multiply(const NumVector & p, const NumVector & q);
NumMatrix skew_symmetric(const NumVector & v);
NumVector quat_normalize(const NumVector & quaternion);
NumMatrix quat_delta_mtx(const NumVector & quaternion, double dt_2);
NumVector acceleration_quat(const NumVector & a);
NumVector magnetometer_quat(const NumVector & l);
NumVector declinator_quat(double declination);
NumVector quat_conjugate(const NumVector & quaternion);
NumVector identity_quaternion();

NumVector lerp(const NumVector & q, const NumVector & p, double alpha);
NumVector slerp(const NumVector & q, const NumVector & p, double alpha);

NumMatrix ddcm_dqs_tr(const NumVector & quaternion);
NumMatrix ddcm_dqx_tr(const NumVector & quaternion);
NumMatrix ddcm_dqy_tr(const NumVector & quaternion);
NumMatrix ddcm_dqz_tr(const NumVector & quaternion);

}


#endif // QUATERNIONS_H
