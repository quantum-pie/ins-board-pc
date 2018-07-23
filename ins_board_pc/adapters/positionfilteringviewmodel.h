#ifndef POSITIONFILTERINGVIEWMODEL_H
#define POSITIONFILTERINGVIEWMODEL_H

#include "adapters/adapter.h"
#include "eigenaux.h"
#include "filtering/public_interfaces/IPositionFilter.h"
#include "packets.h"
#include "qualitycontrol.h"

class Ellipsoid;

/*!
 * @brief Position filtering View Model.
 * Adapted data type for position filtering controllers.
 */
struct PositionFilteringViewModel
{
    Ellipsoid ellip;        //!< Ellipsoid.
    Vector3D pos;           //!< Cartesian position vector.
    double ground_speed;    //!< Ground speed.
    double track_angle;     //!< Track angle.
};

/*!
 * @brief Adapter of IPositionFilter.
 */
template<>
struct Adapter<IPositionFilter, PositionFilteringViewModel>
{
    /*!
     * @brief Convert IPositionFilter to PositionFilteringViewModel.
     * @param filter IOrientationFilter reference.
     * @return PositionFilteringViewModel instance.
     */
    PositionFilteringViewModel operator()(const IPositionFilter & filter);

    /*!
     * @brief Set capacity of the adapter's acumulator.
     * @param new_capacity capacity.
     */
    void set_accumulator_capacity(std::size_t new_capacity);

private:
    QualityControl<Vector3D> speed_accum;
};

/*!
 * @brief Adapter of FilteredPacket.
 */
template<>
struct Adapter<FilteredPacket, PositionFilteringViewModel>
{
    /*!
     * @brief Convert FilteredPacket to PositionFilteringViewModel.
     * @param packet FilteredPacket reference.
     * @return PositionFilteringViewModel instance.
     */
    PositionFilteringViewModel operator()(const FilteredPacket & packet);

    /*!
     * @brief Set capacity of the adapter's acumulator.
     * \param new_capacity capacity.
     */
    void set_accumulator_capacity(std::size_t new_capacity);
};

#endif // POSFILTERINGVIEWMODEL_H
