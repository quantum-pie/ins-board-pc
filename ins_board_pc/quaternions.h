/*! \file quaternions.h
  */

#ifndef QUATERNIONS_H
#define QUATERNIONS_H

#include "eigenaux.h"

//! Module with useful quaternion-related utilities.
namespace qutils
{

/*!
 * \brief Convert quaternion to Euler angles.
 * \param quaternion input quaternion vector.
 * \param[out] roll roll angle.
 * \param[out] pitch pitch angle.
 * \param[out] yaw yaw angle.
 */
NumVector quat_to_rpy(const NumVector & quaternion);

/*!
 * \brief Convert quaternion to transposed direction cosine matrix.
 * \param quaternion input quaternion vector.
 * \return transposed direction cosine matrix.
 */
NumMatrix quaternion_to_dcm_tr(const NumVector & quaternion);

/*!
 * \brief Convert quaternion to direction cosine matrix.
 * \param quaternion input quaternion vector.
 * \return direction cosine matrix.
 */
NumMatrix quaternion_to_dcm(const NumVector & quaternion);

/*!
 * \brief Calculate quaternions product.
 * \param p left-hand quaternion.
 * \param q right-hand quaternion.
 * \return p * q.
 */
NumVector quat_multiply(const NumVector & p, const NumVector & q);

/*!
 * \brief Convert 3d vector to skew-symmetric matrix.
 * \param v input vector.
 * \return skew-symmetric matrix.
 */
NumMatrix skew_symmetric(const NumVector & v);

/*!
 * \brief Normalize quaternion.
 * \param quaternion quaternion vector to normalize.
 * \return normalized quaternion.
 */
NumVector quat_normalize(const NumVector & quaternion);

/*!
 * \brief Convert quaternion to vector differential matrix.
 * \param quaternion quaternion vector.
 * \param dt_2 half of elapsed time.
 * \return vector differential matrix.
 */
NumMatrix quat_delta_mtx(const NumVector & quaternion, double dt_2);

/*!
 * \brief Convert acceleration vector to quaternion.
 * \param a acceleration vector.
 * \return quaternion.
 */
NumVector acceleration_quat(const NumVector & a);

/*!
 * \brief Convert magnetic field vector to quaternion.
 * \param l magnetic field vector.
 * \return quaternion.
 */
NumVector magnetometer_quat(const NumVector & l);

/*!
 * \brief Convert magnetic declination to quaternion.
 * \param declination magnetic declination.
 * \return quaternion.
 */
NumVector declinator_quat(double declination);

/*!
 * \brief Calculate quaternion conjugate.
 * \param quaternion input quaternion.
 * \return quaternion conjugate.
 */
NumVector quat_conjugate(const NumVector & quaternion);

/*!
 * \brief Create identity quaternion.
 * \return identity quaternion.
 */
NumVector identity_quaternion();

/*!
 * \brief Complete quaternion linear interpolation.
 * \param q first quaternion.
 * \param p second quatrnion.
 * \param alpha weight of the first quaternion (0 - 1).
 * \return interpolated quaternion.
 */
NumVector lerp(const NumVector & q, const NumVector & p, double alpha);

/*!
 * \brief Complete quaternion spherical linear interpolation.
 * \param q first quaternion.
 * \param p second quatrnion.
 * \param alpha weight of the first quaternion (0 - 1).
 * \return interpolated quaternion.
 */
NumVector slerp(const NumVector & q, const NumVector & p, double alpha);

/*!
 * \brief Calculate derivative of transposed direction cosine matrix with respect to qs given quaternion.
 * \param quaternion input quaternion.
 * \return derivative of DCM with respect to qs.
 */
NumMatrix ddcm_dqs_tr(const NumVector & quaternion);

/*!
 * \brief Calculate derivative of transposed direction cosine matrix with respect to qx given quaternion.
 * \param quaternion input quaternion.
 * \return derivative of DCM with respect to qx.
 */
NumMatrix ddcm_dqx_tr(const NumVector & quaternion);

/*!
 * \brief Calculate derivative of transposed direction cosine matrix with respect to qy given quaternion.
 * \param quaternion input quaternion.
 * \return derivative of DCM with respect to qy.
 */
NumMatrix ddcm_dqy_tr(const NumVector & quaternion);

/*!
 * \brief Calculate derivative of transposed direction cosine matrix with respect to qz given quaternion.
 * \param quaternion input quaternion.
 * \return derivative of DCM with respect to qz.
 */
NumMatrix ddcm_dqz_tr(const NumVector & quaternion);

}


#endif // QUATERNIONS_H
