#ifndef IKALMANORIENTATIONATTR_H
#define IKALMANORIENTATIONATTR_H

/*!
 * @brief Kalman orientation filter interface.
 */
struct IKalmanOrientationAttr
{
    /*!
     * @brief Set process noise gyroscope standard deviation.
     * @param std standard deviation.
     */
    void set_proc_gyro_std(double std)
    {
        do_set_proc_gyro_std(std);
    }

    /*!
     * @brief Set process noise gyroscope bias standard deviation.
     * @param std standard deviation.
     */
    void set_proc_gyro_bias_std(double std)
    {
        do_set_proc_gyro_bias_std(std);
    }

    /*!
     * @brief Set accelerometer measurements standard deviation.
     * @param std standard deviation.
     */
    void set_meas_accel_std(double std)
    {
        do_set_meas_accel_std(std);
    }

    /*!
     * @brief Set magnetometer measurements standard deviation.
     * @param std standard deviation.
     */
    void set_meas_magn_std(double std)
    {
        do_set_meas_magn_std(std);
    }

    /*!
     * @brief Set initial qs estimate standard deviation.
     * @param std standard deviation.
     */
    void set_init_qs_std(double std)
    {
        do_set_init_qs_std(std);
    }

    /*!
     * @brief Set initial qx estimate standard deviation.
     * @param std standard deviation.
     */
    void set_init_qx_std(double std)
    {
        do_set_init_qx_std(std);
    }

    /*!
     * @brief Set initial qy estimate standard deviation.
     * @param std standard deviation.
     */
    void set_init_qy_std(double std)
    {
        do_set_init_qy_std(std);
    }

    /*!
     * @brief Set initial qz estimate standard deviation.
     * @param std standard deviation.
     */
    void set_init_qz_std(double std)
    {
        do_set_init_qz_std(std);
    }

    /*!
     * @brief Set initial bias estimate standard deviation.
     * @param std standard deviation.
     */
    void set_init_bias_std(double std)
    {
        do_set_init_bias_std(std);
    }

    /*!
     * @brief Get process noise gyroscope bias standard deviation.
     * @return standard deviation.
     */
    double get_proc_gyro_std() const
    {
        return do_get_proc_gyro_std();
    }

    /*!
     * @brief Get process noise gyroscope bias standard deviation.
     * @return standard deviation.
     */
    double get_proc_gyro_bias_std() const
    {
        return do_get_proc_gyro_bias_std();
    }

    /*!
     * @brief Get accelerometer measurements standard deviation.
     * @return standard deviation.
     */
    double get_meas_accel_std() const
    {
        return do_get_meas_accel_std();
    }

    /*!
     * @brief Get magnetometer measurements standard deviation.
     * @return standard deviation.
     */
    double get_meas_magn_std() const
    {
        return do_get_meas_magn_std();
    }

    /*!
     * @brief Get initial qs estimate standard deviation.
     * @return standard deviation.
     */
    double get_init_qs_std() const
    {
        return do_get_init_qs_std();
    }

    /*!
     * @brief Get initial qx estimate standard deviation.
     * @return standard deviation.
     */
    double get_init_qx_std() const
    {
        return do_get_init_qx_std();
    }

    /*!
     * @brief Get initial qy estimate standard deviation.
     * @return standard deviation.
     */
    double get_init_qy_std() const
    {
        return do_get_init_qy_std();
    }

    /*!
     * @brief Get initial qz estimate standard deviation.
     * @return standard deviation.
     */
    double get_init_qz_std() const
    {
        return do_get_init_qz_std();
    }

    /*!
     * @brief Get initial bias estimate standard deviation.
     * @return standard deviation.
     */
    double get_init_bias_std() const
    {
        return do_get_init_bias_std();
    }

    /*!
     * @brief Class destructor.
     */
    virtual ~IKalmanOrientationAttr() = default;

private:
    virtual void do_set_proc_gyro_std(double std) = 0;
    virtual void do_set_proc_gyro_bias_std(double std) = 0;
    virtual void do_set_meas_accel_std(double std) = 0;
    virtual void do_set_meas_magn_std(double std) = 0;
    virtual void do_set_init_qs_std(double std) = 0;
    virtual void do_set_init_qx_std(double std) = 0;
    virtual void do_set_init_qy_std(double std) = 0;
    virtual void do_set_init_qz_std(double std) = 0;
    virtual void do_set_init_bias_std(double std) = 0;

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

#endif // IKALMANORIENTATIONATTR_H
