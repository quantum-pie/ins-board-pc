#ifndef IPOSITIONPROVIDERCORE_H
#define IPOSITIONPROVIDERCORE_H

#include "eigenaux.h"

class Ellipsoid;

struct IPositionProviderCore
{
    /*!
     * @brief Class destructor.
     */
    virtual ~IPositionProviderCore() = default;

    virtual Vector3D do_get_cartesian() const = 0;
    virtual Ellipsoid do_get_ellipsoid() const = 0;
    virtual Vector3D do_get_velocity() const = 0;
    virtual Vector3D do_get_acceleration() const = 0;
};

#endif // IPOSITIONPROVIDERCORE_H
