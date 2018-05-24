#ifndef IKALMANPOSITIONATTR_H
#define IKALMANPOSITIONATTR_H

/*!
 * @brief Kalman position filter interface.
 */
struct IKalmanPositionAttr
{
    /*!
     * @brief Set process noise acceleration standard deviation.
     * @param std standard deviation.
     */
    void set_proc_accel_std(double std)
    {
        do_set_proc_accel_std(std);
    }

    /*!
     * @brief Set measured position CEP.
     * @param std GPS CEP parameter.
     */
    void set_meas_pos_std(double std)
    {
        do_set_meas_pos_std(std);
    }

    /*!
     * @brief Set measured velocity standard deviation.
     * @param std standard deviation.
     */
    void set_meas_vel_std(double std)
    {
        do_set_meas_vel_std(std);
    }

    /*!
     * @brief Set initial position estimate standard deviation.
     * @param std standard deviation.
     */
    void set_init_pos_std(double std)
    {
        do_set_init_pos_std(std);
    }

    /*!
     * @brief Set initial velocity estimate standard deviation.
     * @param std standard deviation.
     */
    void set_init_vel_std(double std)
    {
        do_set_init_vel_std(std);
    }

    /*!
     * @brief Set initial acceleration estimate standard deviation.
     * @param std standard deviation.
     */
    void set_init_accel_std(double std)
    {
        do_set_init_accel_std(std);
    }

    /*!
     * @brief Get process noise acceleration standard deviation.
     * @return standard deviation.
     */
    double get_proc_accel_std() const
    {
        return do_get_proc_accel_std();
    }

    /*!
     * @brief Get measured position CEP.
     * @return GPS CEP parameter.
     */
    double get_meas_pos_std() const
    {
        return do_get_meas_pos_std();
    }

    /*!
     * @brief Get measured velocity standard deviation.
     * @return standard deviation.
     */
    double get_meas_vel_std() const
    {
        return do_get_meas_vel_std();
    }

    /*!
     * @brief Get initial position estimate standard deviation.
     * @return standard deviation.
     */
    double get_init_pos_std() const
    {
        return do_get_init_pos_std();
    }

    /*!
     * @brief Get initial velocity estimate standard deviation.
     * @return standard deviation.
     */
    double get_init_vel_std() const
    {
        return do_get_init_vel_std();
    }

    /*!
     * @brief Get initial acceleration estimate standard deviation.
     * @return standard deviation.
     */
    double get_init_accel_std() const
    {
        return do_get_init_accel_std();
    }

    /*!
     * @brief Class destructor.
     */
    virtual ~IKalmanPositionAttr() = default;

private:
    virtual void do_set_proc_accel_std(double std) = 0;
    virtual void do_set_meas_pos_std(double std) = 0;
    virtual void do_set_meas_vel_std(double std) = 0;
    virtual void do_set_init_pos_std(double std) = 0;
    virtual void do_set_init_vel_std(double std) = 0;
    virtual void do_set_init_accel_std(double std) = 0;

    virtual double do_get_proc_accel_std() const = 0;
    virtual double do_get_meas_pos_std() const = 0;
    virtual double do_get_meas_vel_std() const = 0;
    virtual double do_get_init_pos_std() const = 0;
    virtual double do_get_init_vel_std() const = 0;
    virtual double do_get_init_accel_std() const = 0;
};

#endif // IKALMANPOSITIONATTR_H
