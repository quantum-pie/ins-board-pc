#ifndef IORIENTATIONPROVIDER_H
#define IORIENTATIONPROVIDER_H

#include "eigenaux.h"
#include "quaternion.h"

struct IOrientationProvider
{
    /*!
     * @brief Get current orientation quaternion.
     * @return vector representing quaternion.
     */
    quat::Quaternion get_orientation_quaternion() const
    {
        return do_get_orientation_quaternion();
    }

    /*!
     * @brief Get current gyroscope bias.
     * @return gyroscope bias vector.
     */
    Vector3D get_gyro_bias() const
    {
        return do_get_gyro_bias();
    }

    /*!
     * @brief Class desctructor.
     */
    virtual ~IOrientationProvider() = default;

private:
    virtual quat::Quaternion do_get_orientation_quaternion() const = 0;
    virtual Vector3D do_get_gyro_bias() const = 0;
};

#endif // IORIENTATIONPROVIDER_H
