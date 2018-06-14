#ifndef RAWCONTROLLER_H
#define RAWCONTROLLER_H

#include "views/base/IBaseView.h"
#include "controllers/direct/observablebase.h"
#include "controllers/direct/runningflag.h"

#include <QObject>

class RawPacket;
class QPushButton;

class RawController : public QObject,
                      public ObservableBase<IRawView>,
                      RunningFlag
{
    Q_OBJECT

public:
    explicit RawController(const QPushButton * enable_button = nullptr);

    using RunningFlag::is_running;
    using RunningFlag::set_running;

public slots:
    void handle_input(const RawPacket & z);
    void handle_enable(bool en);
};

#endif // RAWCONTROLLER_H
