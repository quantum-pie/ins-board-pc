#ifndef KALMANORIENTATIONFILTERBASE_H
#define KALMANORIENTATIONFILTERBASE_H

#include "IKalmanOrientationFilter.h"
#include "quatfwd.h"
#include "qualitycontrol.h"
#include "earth.h"
#include "IFilterBase.h"

#include <boost/date_time/gregorian/gregorian.hpp>

class KalmanOrientationFilterBase;

template<>
struct FilterBaseTraits<KalmanOrientationFilterBase>
{
    static constexpr std::size_t state_size { 7 };
    static constexpr std::size_t measurement_size { 6 };

    using state_type = StaticVector<state_size>;
    using meas_type = StaticVector<measurement_size>;

    using F_type = StaticMatrix<state_size, state_size>;
    using P_type = F_type;
    using Q_type = F_type;
    using R_type = StaticMatrix<measurement_size, measurement_size>;
    using H_type = StaticMatrix<measurement_size, state_size>;
    using V_type = quat::skew_type;
    using D_type = quat::delta_type;
};

class KalmanOrientationFilterBase : virtual public IKalmanOrientationFilter,
                                    public IFilterBase<KalmanOrientationFilterBase>
{
public:
    explicit KalmanOrientationFilterBase(const Ellipsoid & ellip = Ellipsoid::WGS84);
    ~KalmanOrientationFilterBase() override;

    using thy_traits = FilterBaseTraits<KalmanOrientationFilterBase>;

    using state_type = typename thy_traits::state_type;
    using meas_type = typename thy_traits::meas_type;

    using F_type = typename thy_traits::F_type;
    using P_type = typename thy_traits::P_type;
    using Q_type = typename thy_traits::Q_type;
    using R_type = typename thy_traits::R_type;
    using H_type = typename thy_traits::H_type;
    using V_type = typename thy_traits::V_type;
    using D_type = typename thy_traits::D_type;

private:
    friend class IFilterBase<KalmanOrientationFilterBase>;

    meas_type do_get_true_measurement(const FilterInput & z) const;
    meas_type do_get_predicted_measurement(const Vector3D & geo, const boost::gregorian::date & day) const;

    state_type do_get_state() const;
    void do_set_state(const state_type & st);

    P_type do_get_cov() const;
    void do_set_cov(const P_type & P);

    Vector3D do_get_geodetic(const FilterInput & z) const;

    bool do_is_initialized() const;
    bool do_is_ready_to_initialize() const;
    void do_initialize(const FilterInput & z);
    void do_accumulate(const FilterInput & z);

    /*!
     * @brief Create state transition matrix (F).
     * @param z filter input reference.
     * @return state transition matrix.
     */
    F_type do_create_transition_mtx(const FilterInput & z) const;

    /*!
     * @brief Create initial state estimate covariance matrix (P).
     * @param dt time elapsed since the last step.
     * @return state transition matrix.
     */
    P_type do_create_init_cov_mtx() const;

    /*!
     * @brief Create process noise covariance matrix (Q).
     * @param dt time elapsed since the last step.
     * @return process noise covariance matrix.
     */
    Q_type do_create_proc_noise_cov_mtx(double dt) const;

    /*!
     * @brief Create measurement noise covariance matrix (R).
     * @param geo geodetic coordinates.
     * @param mag_magn magnetic field magnitude.
     * @return measurement noise covariance matrix.
     */
    R_type do_create_meas_noise_cov_mtx(const Vector3D & geo, const boost::gregorian::date & day) const;

    /*!
     * @brief Create state-to-measurement projection matrix (H).
     * @param geo geodetic coordinates.
     * @param earth_model Earth model.
     * @return state-to-measurement projection matrix.
     */
    H_type do_create_meas_proj_mtx(const Vector3D & geo, const boost::gregorian::date & day) const;

    static constexpr std::size_t accum_size { 500 };

    void do_reset() override;

    quat::Quaternion do_get_orientation_quaternion() const override;
    Vector3D do_get_gyro_bias() const override;

    void do_set_proc_gyro_std(double std) override;
    void do_set_proc_gyro_bias_std(double std) override;
    void do_set_meas_accel_std(double std) override;
    void do_set_meas_magn_std(double std) override;
    void do_set_init_qs_std(double std) override;
    void do_set_init_qx_std(double std) override;
    void do_set_init_qy_std(double std) override;
    void do_set_init_qz_std(double std) override;
    void do_set_init_bias_std(double std) override;

    double do_get_proc_gyro_std() const override;
    double do_get_proc_gyro_bias_std() const override;
    double do_get_meas_accel_std() const override;
    double do_get_meas_magn_std() const override;
    double do_get_init_qs_std() const override;
    double do_get_init_qx_std() const override;
    double do_get_init_qy_std() const override;
    double do_get_init_qz_std() const override;
    double do_get_init_bias_std() const override;

    const Earth earth_model;            //!< Reference Earth model.
    state_type x;
    P_type P;

    QualityControl<Vector3D> bias_ctrl;
    bool initialized;

    struct
    {
        struct
        {
            double gyro_std;        //!< Process noise gyroscope standard deviation.
            double gyro_bias_std;   //!< Process noise gyroscope bias standard deviation.
        } proc_params;

        struct
        {
            double accel_std;           //!< accelerometer measurements std.
            double magn_std;            //!< magnetometer measurements std.
        } meas_params;

        struct
        {
            double qs_std;      //!< Initial qs estimate standard deviation.
            double qx_std;      //!< Initial qx estimate standard deviation.
            double qy_std;      //!< Initial qy estimate standard deviation.
            double qz_std;      //!< Initial qz estimate standard deviation.
            double bias_std;    //!< Initial gyro bias estimate standard deviation.
        } init_params;
    } params;
};

#endif // KALMANORIENTATIONFILTERBASE_H
