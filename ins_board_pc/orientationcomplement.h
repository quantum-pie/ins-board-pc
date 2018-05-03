/*! \file orientationcomplement.h
  */

#ifndef ORIENTATIONCOMPLEMENT_H
#define ORIENTATIONCOMPLEMENT_H

#include "IComplementOrientationFilter.h"
#include "qualitycontrol.h"
#include "earth.h"

/*!
 * @brief Concrete complementary orientation filter class.
 */
class OrientationCF final : public IComplementOrientationFilter
{
public:
    /*!
     * @brief Reduced filter input structure for convenience.
     */
	struct FInput
	{
		Vector3D w;
		Vector3D a;
		Vector3D m;
		double dt;
	};

    /*!
     * @brief Filter parameters structure.
     */
    struct FilterParams
    {
        double static_accel_gain;   //!< Static accelerometer measurements gain.
        double static_magn_gain;    //!< Static magnetometer measurements gain.
        double bias_gain;			//!< Gyroscope bias gain.
        std::size_t accum_capacity;	//!< Capacity of measurements accumulator.
    };

    /*!
     * @brief Class constructor.
     * @param params filter parameters structure.
     */
    OrientationCF(const FilterParams & params);

    /*!
     * @brief Class destructor.
     */
    ~OrientationCF() override;

    /* IComplementOrientationFilter interface implementation */
    void step(const FilterInput & z) override;
    void reset() override;

    Quaternion get_orientation_quaternion() const override;
    Vector3D get_gyro_bias() const override;

    void set_static_accel_gain(double gain) override;
    void set_static_magn_gain(double gain) override;
    void set_bias_gain(double gain) override;

    double get_static_accel_gain() const override;
    double get_static_magn_gain() const override;
    double get_bias_gain() const override;

private:
    /*!
     * @brief Convert generic filter input to reduced one.
     * @param z filter input.
     * @return reduced filter input.
     */
    FInput adopt_input(const FilterInput & z);

    /*!
     * @brief Step of uninitialized filter.
     * @param z filter input.
     */
    void step_initialized(const FInput & z);

    /*!
     * @brief Step of initialized filter.
     * @param z filter input.
     */
    void step_uninitialized(const FInput & z);

    /*!
     * @brief Initialize filter.
     * @param z filter input.
     */
    void initialize(const FInput & z);

    static constexpr int state_size { 7 };        	        //!< Size of state vector.

    /* Useful type aliases */
    using state_type = StaticVector<state_size>;
    using F_type = StaticMatrix<state_size, state_size>;
    using V_type = Quaternion::skew_type;
    using K_type = Quaternion::delta_type;

    /*!
     * @brief normalize filter state.
     */
    void normalize_state();

    /*!
     * @brief Calculate dynamic accelerometer measurements gain.
     * @param accel measured acceleration.
     * @return dynamic gain.
     */
    double calculate_gain(const Vector3D & accel) const;

    /*!
     * @brief Calculate quaternion from accelerometer and magnetometer readings.
     * @param accel measured acceleration.
     * @param accel measured magnetic field.
     * @return measured quaternion.
     */
    Quaternion measured_quaternion(const Vector3D & accel, const Vector3D & magn) const;

    bool is_initialized;                    //!< Filter is initialized flag.
    FilterParams params;                    //!< Filter parameters structure.
    QualityControl<Vector3D> bias_ctrl;     //!< Gyroscope bias controller.
    state_type x;                           //!< Filter state.

    const Earth earth_model;                //!< Reference Earth model.
};

#endif
