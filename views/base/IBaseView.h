#ifndef BASEVIEW_H
#define BASEVIEW_H

class FilterInput;

/*!
 * @brief Base model view.
 * @tparam Model model type.
 */
template<typename Model>
struct IBaseView
{
    //! Model alias.
    using ViewModel = Model;

    /*!
     * @brief Destructor.
     */
    virtual ~IBaseView() = default;

    /*!
     * @brief Update view.
     * @param pvd Model reference.
     */
    virtual void update(const ViewModel & pvd) = 0;

    /*!
     * @brief Clear view.
     */
    virtual void clear() = 0;
};

struct OrientationFilteringViewModel;
struct PositionFilteringViewModel;
struct RawViewModel;
struct FilteredPacket;
class MagnCalibrator;

//! Position view alias.
using IPositionView = IBaseView<PositionFilteringViewModel>;

//! Orientation view alias.
using IOrientationView = IBaseView<OrientationFilteringViewModel>;

//! Raw data view alias.
using IRawView = IBaseView<RawViewModel>;

//! Calibration view alias.
using ICalibrationView = IBaseView<MagnCalibrator>;

#endif // BASEORIENTATIONVIEW_H
