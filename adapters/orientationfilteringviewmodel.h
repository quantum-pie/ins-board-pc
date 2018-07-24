#ifndef ORIENTATIONFILTERINGVIEWMODEL_H
#define ORIENTATIONFILTERINGVIEWMODEL_H

#include "adapters/adapter.h"
#include "eigenaux.h"
#include "filtering/public_interfaces/IOrientationFilter.h"
#include "packets.h"
#include "qualitycontrol.h"

#include <QQuaternion>

/*!
 * @brief Orientation filtering View Model.
 * Adapted data type for orientation filtering controllers.
 */
struct OrientationFilteringViewModel
{
    QQuaternion q;      //!< Orientation quaternion.
    Vector3D rpy;       //!< Euler angles.
    Vector3D rpy_std;   //!< Euler angles standard deviation.
    Vector3D rpy_mean;  //!< Euler angles mean.
};

/*!
 * @brief Adapter of IOrientationFilter.
 */
template<>
struct Adapter<IOrientationFilter, OrientationFilteringViewModel>
{
    /*!
     * @brief Convert IOrientationFilter to OrientationFilteringViewModel.
     * @param filter IOrientationFilter reference.
     * @return OrientationFilteringViewModel instance.
     */
    OrientationFilteringViewModel operator()(const IOrientationFilter & filter);

    /*!
     * @brief Set capacity of the adapter's acumulator.
     * @param new_capacity capacity.
     */
    void set_accumulator_capacity(std::size_t new_capacity);

private:
    QualityControl<Vector3D> std_ctrl;
};

/*!
 * @brief Adapter of FilteredPacket.
 */
template<>
struct Adapter<FilteredPacket, OrientationFilteringViewModel>
{
    /*!
     * @brief Convert FilteredPacket to OrientationFilteringViewModel.
     * @param packet FilteredPacket reference.
     * @return OrientationFilteringViewModel instance.
     */
    OrientationFilteringViewModel operator()(const FilteredPacket & packet);

    /*!
     * @brief Set capacity of the adapter's acumulator.
     * \param new_capacity capacity.
     */
    void set_accumulator_capacity(std::size_t new_capacity);

private:
    QualityControl<Vector3D> std_ctrl;
};

#endif // ORIENTATIONFILTERINGVIEWMODEL_H
