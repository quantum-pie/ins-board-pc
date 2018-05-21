#ifndef IKALMANORIENTATIONATTRGETCORE_H
#define IKALMANORIENTATIONATTRGETCORE_H

struct IKalmanOrientationAttrGetCore
{
    /*!
     * @brief Class destructor.
     */
    virtual ~IKalmanOrientationAttrGetCore() = default;

    virtual double do_get_proc_gyro_std() const = 0;
    virtual double do_get_proc_gyro_bias_std() const = 0;
    virtual double do_get_meas_accel_std() const = 0;
    virtual double do_get_meas_magn_std() const = 0;
    virtual double do_get_init_qs_std() const = 0;
    virtual double do_get_init_qx_std() const = 0;
    virtual double do_get_init_qy_std() const = 0;
    virtual double do_get_init_qz_std() const = 0;
    virtual double do_get_init_bias_std() const = 0;
};

#endif // IKALMANORIENTATIONATTRGETCORE_H
