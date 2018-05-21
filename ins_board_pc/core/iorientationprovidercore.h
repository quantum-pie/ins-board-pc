#ifndef IORIENTATIONPROVIDERCORE_H
#define IORIENTATIONPROVIDERCORE_H

#include "quatfwd.h"

struct IOrientationProviderCore
{
    /*!
     * @brief Class desctructor.
     */
    virtual ~IOrientationProviderCore() = default;

    virtual quat::Quaternion do_get_orientation_quaternion() const = 0;
    virtual Vector3D do_get_gyro_bias() const = 0;
};

#endif // IORIENTATIONPROVIDERCORE_H
