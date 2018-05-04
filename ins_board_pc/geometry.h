/*
 * geometry.h
 *
 *      Author: bigaw
 */

#ifndef INCLUDE_GEOMETRY_H_
#define INCLUDE_GEOMETRY_H_

#include "eigenaux.h"
#include "ellipsoid.h"
#include "quaternion.h"
#include "horizon.h"

/*!
 * @brief This namespace holds various geometrical utilities.
 */
namespace geom
{

/*!
 * @brief Calculate destination point along great circle.
 *
 * This function calculates destination geodetic point after traveling given distance along great circle
 * from specified geodetic point with given starting bearing.
 *
 * @param geo starting geodetic point.
 * @param start_bearing starting bearing.
 * @param distance traveled distance.
 * @param e reference ellipsoid model.
 * @return finish geodetic point.
 */
Vector3D great_circle_destination(const Vector3D & geo, double start_bearing, double distance, const Ellipsoid & e);

/*!
 * @brief Convert ECEF to geodetic coordinates.
 * @param pos ECEF point.
 * @param e reference ellipsoid model.
 * @return geodetic coordinates.
 */
Vector3D cartesian_to_geodetic(const Vector3D & pos, const Ellipsoid & e);

/*!
 * @brief Convert geodetic coordinates to ECEF.
 * @param pos geodetic point.
 * @param e reference ellipsoid model.
 * @return ECEF point.
 */
Vector3D geodetic_to_cartesian(const Vector3D & pos, const Ellipsoid & e);

/*!
 * @brief Convert vector from ECEF to ENU coordinates.
 * @param ecef vector in ECEF.
 * @param geo geodetic coordinates of ENU system.
 * @return vector in ENU.
 */
Vector3D ecef_to_enu(const Vector3D & ecef, const Vector3D & geo);

/*!
 * @brief Convert vector from ENU to ECEF coordinates.
 * @param enu vector in ENU.
 * @param geo geodetic coordinates of ENU system.
 * @return vector in ECEF.
 */
Vector3D enu_to_ecef(const Vector3D & enu, const Vector3D & geo);

/*!
 * @brief Get direction cosine matrix from geodetic coordinates.
 * @param geo geodetic coordinates.
 * @return direction cosine matrix.
 */
Matrix3D geodetic_to_dcm(const Vector3D & geo);

/*!
 * @brief Get partial derivative of DCM with respect do latitude.
 * @param geo geodetic coordinates.
 * @return partial derivative of DCM.
 */
Matrix3D dcm_lat_partial(const Vector3D & geo);

/*!
 * @brief Get partial derivative of DCM with respect do longitude.
 * @param geo geodetic coordinates.
 * @return partial derivative of DCM.
 */
Matrix3D dcm_lon_partial(const Vector3D & geo);

/*!
 * @brief Get partial derivative of geodetic coordinates with respect to ECEF ones.
 * @param geo geodetic coordinates.
 * @param e reference ellipsoid model.
 * @return partial derivatives.
 */
Matrix3D dgeo_dpos(const Vector3D & geo, const Ellipsoid & e);

/*!
 * @brief Get target ground speed.
 * @param ecef_vel target velocity vector in ECEF.
 * @param geo target geodetic coordinates.
 * @return target ground speed in m/s.
 */
double ground_speed(const Vector3D & ecef_vel, const Vector3D & geo);

/*!
 * @brief Get target track angle.
 * @param ecef_vel target velocity vector in ECEF.
 * @param geo target geodetic coordinates.
 * @return target track angle in radians.
 */
double track_angle(const Vector3D & ecef_vel, const Vector3D & geo);

/*!
 * @brief Calculate quaternion which describes orientation of rigid body with given accelerometer readings.
 * @param a accelerometer readings.
 * @return acceleration quaternion.
 */
quat::Quaternion acceleration_quat(const Vector3D & a);

/*!
 * @brief Calculate quaternion which describes orientation of rigid body with given magnetometer readings.
 * @param l magnetometer readings.
 * @return magnetometer quaternion.
 */
quat::Quaternion magnetometer_quat(const Vector3D & l);

/*!
 * @brief Calculate quaternion which describes orientation of rigid body with given magnetometer and accelerometer readings.
 * @param a accelerometer readings.
 * @param m magnetometer readings.
 * @return orientation quaternion.
 */
quat::Quaternion accel_magn_quat(const Vector3D & a, const Vector3D & m);

/*!
 * @brief Align vector relative to horizon.
 * @param vec vector to align.
 * @param hor horizon reference.
 * @return aligned vector.
 */
Vector3D align(const Vector3D & vec, const Horizon & hor);

/*!
 * @brief Align quaternion relative to horizon.
 * @param q quaternion to align.
 * @param hor horizon reference.
 * @return aligned quaternion.
 */
quat::Quaternion align(const quat::Quaternion & q, const Horizon & hor);

/*!
 * @brief Map quaternion to accelerometer measurements.
 * @param q quaternion.
 * @param enu_accel linear body acceleration in ENU system.
 * @param gravity_magn gravity magnitude.
 * @return predicted accelerometer readings.
 */
Vector3D predict_accelerometer(const quat::Quaternion & q, const Vector3D & enu_accel, double gravity_magn);

/*!
 * @brief Map quaternion to magnetometer measurements.
 * @param q quaternion.
 * @param enu_magnetic_field magnetic vector in ENU system.
 * @return predicted megnetometer readings.
 */
Vector3D predict_magnetometer(const quat::Quaternion & q, const Vector3D & enu_magnetic_field);

}

#endif /* INCLUDE_GEOMETRY_H_ */
