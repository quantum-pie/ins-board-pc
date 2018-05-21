#ifndef ISIMPOSITIONFILTER_H
#define ISIMPOSITIONFILTER_H

#include "filtering/public_interfaces/IPositionFilter.h"
#include "core/ISimPositionAttrGetCore.h"
#include "core/ISimPositionAttrSetCore.h"

struct ISimPositionFilter : IPositionFilter,
                            private ISimPositionAttrGetCore,
                            private ISimPositionAttrSetCore
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

    ~ISimPositionFilter() override = default;
};

#endif // ISIMPOSITIONFILTER_H
