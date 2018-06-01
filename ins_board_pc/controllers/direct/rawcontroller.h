#ifndef RAWCONTROLLER_H
#define RAWCONTROLLER_H

#include "views/IBaseView.h"
#include "controllers/direct/observablebase.h"
#include "controllers/direct/runningflag.h"

#include <QObject>

class Receiver;
class RawPacket;
class QPushButton;

class RawController : public QObject,
                      public ObservableBase<IRawView>,
                      RunningFlag
{
    Q_OBJECT

public:
    explicit RawController(const Receiver * receiver, const QPushButton * enable_button = nullptr);

    using RunningFlag::is_running;
    using RunningFlag::set_running;

public slots:
    void handle_input(const RawPacket & z);
};

#endif // RAWCONTROLLER_H
