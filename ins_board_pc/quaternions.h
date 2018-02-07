#ifndef QUATERNIONS_H
#define QUATERNIONS_H

#include "ublasaux.h"

namespace qutils
{

void quat_to_rpy(const NumVector & quaternion, double & roll, double & pitch, double & yaw);

NumMatrix quaternion_to_dcm(const NumVector & quaternion);

NumVector quat_multiply(const NumVector & p, const NumVector & q);

NumMatrix skew_symmetric(const NumVector & v);

NumVector quat_normalize(const NumVector & quaternion);

NumMatrix quat_delta_mtx(const NumVector & quaternion, double dt_2);

NumMatrix ddcm_dqs(const NumVector & quaternion);
NumMatrix ddcm_dqx(const NumVector & quaternion);
NumMatrix ddcm_dqy(const NumVector & quaternion);
NumMatrix ddcm_dqz(const NumVector & quaternion);

}


#endif // QUATERNIONS_H
