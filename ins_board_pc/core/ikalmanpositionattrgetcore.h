#ifndef IKALMANPOSITIONATTRGETCORE_H
#define IKALMANPOSITIONATTRGETCORE_H

/*!
 * @brief Kalman position filter interface.
 */
struct IKalmanPositionAttrGetCore
{
    /*!
     * @brief Class destructor.
     */
    virtual  ~IKalmanPositionAttrGetCore() = default;

    virtual double do_get_proc_accel_std() const = 0;
    virtual double do_get_meas_pos_std() const = 0;
    virtual double do_get_meas_vel_std() const = 0;
    virtual double do_get_init_pos_std() const = 0;
    virtual double do_get_init_vel_std() const = 0;
    virtual double do_get_init_accel_std() const = 0;
};

#endif // IKALMANPOSITIONATTRGETCORE_H
