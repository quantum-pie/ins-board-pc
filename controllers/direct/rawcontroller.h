#ifndef RAWCONTROLLER_H
#define RAWCONTROLLER_H

#include "views/base/IBaseView.h"
#include "controllers/observablebase.h"
#include "controllers/runningflag.h"
#include "adapters/rawviewmodel.h"

#include <QObject>

class RawPacket;
class QPushButton;

/*!
 * @brief The RawController class
 * This is the controller of raw data.
 */
class RawController : public QObject,
                      public ObservableBase<IRawView>,
                      RunningFlag
{
    Q_OBJECT

public:
    /*!
     * @brief RawController constructor.
     * @param enable_button Enable widget (optional).
     */
    explicit RawController(const QPushButton * enable_button = nullptr);

    //! Bring is running check to scope.
    using RunningFlag::is_running;

    //! Bring is runnung setter to scope.
    using RunningFlag::set_running;

public slots:
    /*!
     * @brief Handle raw input.
     * @param z raw input packet.
     */
    void handle_input(const RawPacket & z);

    /*!
     * @brief Handle enable.
     * @param en enable boolean condition.
     */
    void handle_enable(bool en);

private:
    Adapter<RawPacket, RawViewModel> mvm_adapter;
};

#endif // RAWCONTROLLER_H
