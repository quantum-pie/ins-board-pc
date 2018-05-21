#ifndef IKALMANPOSITIONATTRSETCORE_H
#define IKALMANPOSITIONATTRSETCORE_H

/*!
 * @brief Kalman position filter interface.
 */
struct IKalmanPositionAttrSetCore
{
    /*!
     * @brief Class destructor.
     */
    virtual ~IKalmanPositionAttrSetCore() = default;

    virtual void do_set_proc_accel_std(double std) = 0;
    virtual void do_set_meas_pos_std(double std) = 0;
    virtual void do_set_meas_vel_std(double std) = 0;
    virtual void do_set_init_pos_std(double std) = 0;
    virtual void do_set_init_vel_std(double std) = 0;
    virtual void do_set_init_accel_std(double std) = 0;
};

#endif // IKALMANPOSITIONATTRSETCORE_H
