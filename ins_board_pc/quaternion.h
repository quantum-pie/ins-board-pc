/*
 * quaternion.h
 *
 *      Author: bigaw
 */

#ifndef INCLUDE_QUATERNION_H_
#define INCLUDE_QUATERNION_H_

#include "eigenaux.h"

namespace quat
{

/*!
 * @brief Quaternion class.
 */
class Quaternion
{
public:
	using skew_type = StaticMatrix<4, 4>;
	using delta_type = StaticMatrix<4, 3>;
	using vector_form = StaticVector<4>;

	/*!
	 * @brief Class constructor from quaternion elements.
	 * @param qs qs.
	 * @param qx qx.
	 * @param qy qy.
	 * @param qz qz.
	 */
	Quaternion(double qs, double qx, double qy, double qz);

	/*!
	 * @brief Pure imaginary rotation quaternion construction from vector.
	 * @param v vector.
	 */
	Quaternion(const Vector3D & v);

	/*!
	 * @brief Class constructor from vector of quaternion elements.
	 * @param v vector of quaternion elements.
	 */
	Quaternion(const vector_form & v);

	/*!
	 * @brief Convert quaternion to Euler angles.
	 * @return Euler angles vector.
	 */
	Vector3D rpy() const;

	/*!
	 * @brief Convert quaternion to direction cosine matrix.
	 * @return DCM.
	 */
	Matrix3D dcm() const;

	/*!
	 * @brief Convert quaternion to transposed direction cosine matrix.
	 * @return transposed DCM.
	 */
	Matrix3D dcm_tr() const;

	/*!
	 * @brief Convert quaternion to increment matrix.
	 *
	 * This function returns special quaternion increment matrix K. Given this matrix and angular velocity vector w,
	 * one can calculate rigid body quaternion increment, if this body experienced rotation given by w over time 2 * dt_2.
	 * The formula is as follows: delta_q = K(q, dt_2) * w.
	 * @param dt_2 half of the time elapsed.
	 * @return quaternion increment matrix.
	 */
	delta_type delta_mtx(double dt_2) const;

	/*!
	 * @brief Calculate quaternion norm.
	 * @return norm.
	 */
	double norm() const;

	/*!
	 * @brief Normalize this quaternion.
	 * @return reference to this quaternion.
	 */
	Quaternion& normalize();

	/*!
	 * @brief Get quaternion conjugate.
	 * @return conjugate.
	 */
	Quaternion conjugate() const;

	/*!
	 * @brief Get quaternion scalar part.
	 * @return quaternion scalar part.
	 */
	double scalar_part() const;

	/*!
	 * @brief Get quaternion vector part.
	 * @return quaternion vector part.
	 */
	Vector3D vector_part() const;

	/*!
	 * @brief Check if this quaternion is real.
	 * @return true if is real.
	 */
	bool is_real() const;

	/*!
	 * @brief Check if this quaternion is pure imaginary.
	 * @return
	 */
	bool is_pure_imag() const;

	/*!
	 * @brief Calculate derivative of transposed quaternion direction cosine matrix
	 * with respect to first quaternion component qs.
	 * @return derivatives matrix.
	 */
	Matrix3D ddcm_dqs_tr() const;

	/*!
 	* @brief Calculate derivative of transposed quaternion direction cosine matrix
 	* with respect to second quaternion component qx.
	* @return derivatives matrix.
	*/
	Matrix3D ddcm_dqx_tr() const;

	/*!
 	* @brief Calculate derivative of transposed quaternion direction cosine matrix
 	* with respect to third quaternion component qy.
	* @return derivatives matrix.
	*/
	Matrix3D ddcm_dqy_tr() const;

	/*!
	* @brief Calculate derivative of transposed quaternion direction cosine matrix
	* with respect to fourth quaternion component qz.
	* @return derivatives matrix.
	*/
	Matrix3D ddcm_dqz_tr() const;

	/*!
	 * @brief Quaternion product operator.
	 * @param p second operand.
	 * @return product of this quaternion with p.
	 */
	Quaternion operator*(const Quaternion & p) const;

	/*!
	 * @brief Quaternion assignment product operator.
	 * @param p second operand.
	 * @return this quaternion after multiplication with p.
	 */
	Quaternion& operator*=(const Quaternion & p);

	/*!
	 * @brief Quaternion scalar product operator.
	 * @param v scalar operand.
	 * @return product of this quaternion with p.
	 */
	Quaternion operator*(double v) const;

	/*!
	 * @brief Quaternion assignment scalar product operator.
	 * @param v scalar operand.
	 * @return this quaternion after multiplication with p.
	 */
	Quaternion& operator*=(double v);

	/*!
	 * @brief Quaternion addition operator.
	 * @param p second operand.
	 * @return sum of this quaternion with p.
	 */
	Quaternion operator+(const Quaternion & p) const;

	/*!
	 * @brief Quaternion assignment addition operator.
	 * @param p second operand.
	 * @return this quaternion after addition of p.
	 */
	Quaternion& operator+=(const Quaternion & p);

	/*!
	 * @brief Convert quaternion to vector of its components.
	 * @return vector form of quaternion.
	 */
	operator vector_form() const;

	static const Quaternion identity; 						//!< Identity quaternion.

private:
	double qs, qx, qy, qz;									//!< Quaternion components.
};

}

#endif /* INCLUDE_QUATERNION_H_ */
