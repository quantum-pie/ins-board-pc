#ifndef ISIMPOSITIONATTR_H
#define ISIMPOSITIONATTR_H

struct ISimPositionAttr
{
    /*!
     * @brief Set start track angle.
     * @param radians track angle.
     */
    void set_initial_track(double radians)
    {
        do_set_initial_track(radians);
    }

    /*!
     * @brief Set movement speed.
     * @param ms speed in m/s.
     */
    void set_speed(double ms)
    {
        do_set_speed(ms);
    }

    /*!
     * @brief Get current start track angle.
     * @return track angle.
     */
    double get_initial_track() const
    {
        return do_get_initial_track();
    }

    /*!
     * @brief Get current movement speed.
     * @return movement speed in m/s.
     */
    double get_speed() const
    {
        return do_get_speed();
    }

    virtual ~ISimPositionAttr() = default;

private:
    virtual void do_set_initial_track(double radians) = 0;
    virtual void do_set_speed(double ms) = 0;

    virtual double do_get_initial_track() const = 0;
    virtual double do_get_speed() const = 0;
};

#endif // ISIMPOSITIONATTR_H
