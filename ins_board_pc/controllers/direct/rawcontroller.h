#ifndef RAWCONTROLLER_H
#define RAWCONTROLLER_H

#include "views/IBaseView.h"
#include "controllers/direct/observablebase.h"

#include <QObject>

class Receiver;
class RawPacket;

class RawController : public QObject, public ObservableBase<IRawView>
{
    Q_OBJECT

public:
    RawController(const Receiver * receiver);

public slots:
    void handle_input(const RawPacket & z);
};

#endif // RAWCONTROLLER_H
