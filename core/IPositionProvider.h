#ifndef IPOSITIONPROVIDER_H
#define IPOSITIONPROVIDER_H

#include "eigenaux.h"
#include "ellipsoid.h"

/*!
 * @brief Position provider interface.
 */
struct IPositionProvider
{
    /*!
     * @brief Get ECEF coordinates vector.
     * @return position vector.
     */
    Vector3D get_position() const
    {
        return do_get_position();
    }

    /*!
     * @brief Get underlying Earth ellipsoid model.
     * @return ellipsoid model reference.
     */
    Ellipsoid get_ellipsoid() const
    {
        return do_get_ellipsoid();
    }

    /*!
     * @brief Get ECEF velocity vector.
     * @return velocity vector.
     */
    Vector3D get_velocity() const
    {
        return do_get_velocity();
    }

    /*!
     * @brief Get ECEF acceleration vector.
     * @return acceleration vector.
     */
    Vector3D get_acceleration() const
    {
        return do_get_acceleration();
    }

    /*!
     * @brief Class destructor.
     */
    virtual ~IPositionProvider() = default;

private:
    virtual Vector3D do_get_position() const = 0;
    virtual Ellipsoid do_get_ellipsoid() const = 0;
    virtual Vector3D do_get_velocity() const = 0;
    virtual Vector3D do_get_acceleration() const = 0;
};

#endif // IPOSITIONPROVIDER_H
