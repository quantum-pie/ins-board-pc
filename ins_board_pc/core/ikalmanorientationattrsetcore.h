#ifndef IKALMANORIENTATIONATTRSETCORE_H
#define IKALMANORIENTATIONATTRSETCORE_H

struct IKalmanOrientationAttrSetCore
{
    /*!
     * @brief Class destructor.
     */
    virtual ~IKalmanOrientationAttrSetCore() = default;

    virtual void do_set_proc_gyro_std(double std) = 0;
    virtual void do_set_proc_gyro_bias_std(double std) = 0;
    virtual void do_set_meas_accel_std(double std) = 0;
    virtual void do_set_meas_magn_std(double std) = 0;
    virtual void do_set_init_qs_std(double std) = 0;
    virtual void do_set_init_qx_std(double std) = 0;
    virtual void do_set_init_qy_std(double std) = 0;
    virtual void do_set_init_qz_std(double std) = 0;
    virtual void do_set_init_bias_std(double std) = 0;
};

#endif // IKALMANORIENTATIONATTRSETCORE_H
