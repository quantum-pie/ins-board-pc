#ifndef FILTERIMPL_H
#define FILTERIMPL_H

#include "eigenaux.h"
#include <cstddef>

namespace impl
{

extern const std::size_t pure_pos_state_size;
extern const std::size_t pure_pos_measurement_size;
extern const std::size_t pure_ori_state_size;
extern const std::size_t pure_ori_measurement_size;

extern const std::size_t mixed_state_size;
extern const std::size_t mixed_measurement_size;

using pure_pos_F_type = StaticMatrix<pure_pos_state_size, pure_pos_state_size>;
using pure_pos_P_type = pure_pos_F_type;
using pure_pos_Q_type = pure_pos_F_type;
using pure_pos_R_type = StaticMatrix<pure_pos_measurement_size, pure_pos_measurement_size>;
using pure_pos_H_type = StaticMatrix<pure_pos_measurement_size, pure_pos_state_size>;

using pure_ori_F_type = StaticMatrix<pure_ori_state_size, pure_ori_state_size>;
using pure_ori_P_type = pure_ori_F_type;
using pure_ori_Q_type = pure_ori_F_type;
using pure_ori_R_type = StaticMatrix<pure_ori_measurement_size, pure_ori_measurement_size>;
using pure_ori_H_type = StaticMatrix<pure_ori_measurement_size, pure_ori_state_size>;

using mixed_F_type = StaticMatrix<mixed_state_size, mixed_state_size>;
using mixed_P_type = mixed_F_type;
using mixed_Q_type = mixed_F_type;
using mixed_R_type = StaticMatrix<mixed_measurement_size, mixed_measurement_size>;
using mixed_H_type = StaticMatrix<mixed_measurement_size, mixed_state_size>;

/*!
 * @brief Create state transition matrix (F).
 * @param dt time elapsed since the last step.
 * @return state transition matrix.
 */
pure_pos_F_type create_transition_mtx(double dt);

/*!
 * @brief Create process noise covariance matrix (Q).
 * @param dt time elapsed since the last step.
 * @return process noise covariance matrix.
 */
pure_pos_Q_type create_proc_noise_cov_mtx(double dt);

/*!
 * @brief Create measurement noise covariance matrix (R).
 * @param geo geodetic coordinates vector.
 * @return measurement noise covariance matrix.
 */
pure_pos_R_type create_meas_noise_cov_mtx(const Vector3D & geo) const;

/*!
 * @brief Create ENU system measurement covariance matrix.
 * @return local ENU measurement noise covariance matrix.
 */
Matrix3D create_local_cov_mtx() const;

/*!
 * @brief Create state-to-measurement projection matrix (H).
 * @return state-to-measurement projection matrix.
 */
pure_pos_H_type create_meas_proj_mtx() const;

}

#endif // FILTERIMPL_H
