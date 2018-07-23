#ifndef FILTERINGCONTROLLERCOMMON_H
#define FILTERINGCONTROLLERCOMMON_H

#include "controllers/runningflag.h"

#include <QObject>

class FilterInput;

/*!
 * @brief The FilteringControllerCommon class
 * This is the non-template part of any filtering controller instance.
 */
class FilteringControllerCommon : public QObject, RunningFlag
{
    Q_OBJECT

public:
    /*!
     * @brief FilteringControllerCommon constructor.
     * @param already_running initial filtering status.
     */
    FilteringControllerCommon(bool already_running);

    /*!
      * @brief FilteringControllerCommon destructor.
      */
    virtual ~FilteringControllerCommon() = default;

    //! Bring is running check to scope.
    using RunningFlag::is_running;

    //! Bring is runnung setter to scope.
    using RunningFlag::set_running;

    /*!
     * @brief Check if filtering is enabled.
     * @return true of is enabled.
     */
    bool filtering_is_enabled();

    /*!
     * @brief Enable filtering.
     */
    void enable_filtering();

    /*!
     * @brief Disable filtering.
     */
    void disable_filtering();

public slots:
    /*!
     * @brief Handle filtering start.
     * @param en start boolean condition.
     */
    virtual void handle_start(bool en) = 0;

    /*!
     * @brief Handle filter input.
     * @param z filter input.
     */
    virtual void handle_input(const FilterInput & z) = 0;

    /*!
     * @brief Set adapter's accumulator capacity.
     * @param cap new capacity.
     */
    virtual void set_accum_capacity(const QString & cap) = 0;

    /*!
     * @brief Clear associated views.
     */
    virtual void clear_plots() = 0;

private:
    bool filtering_enabled;
};

#endif // FILTERINGCONTROLLERCOMMON_H
