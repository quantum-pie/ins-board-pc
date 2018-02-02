#ifndef QUATERNIONS_H
#define QUATERNIONS_H

#include "ublastypes.h"

namespace qutils
{

void quat_to_rpy(const NumVector & quaternion, double & roll, double & pitch, double & yaw);

NumMatrix quaternion_to_dcm(const NumVector & quaternion);

NumVector quat_multiply(const NumVector & p, const NumVector & q);

void quat_normalize(NumVector & quaternion);

NumMatrix ddcm_dqs(const NumVector & quaternion);
NumMatrix ddcm_dqx(const NumVector & quaternion);
NumMatrix ddcm_dqy(const NumVector & quaternion);
NumMatrix ddcm_dqz(const NumVector & quaternion);

}


#endif // QUATERNIONS_H
