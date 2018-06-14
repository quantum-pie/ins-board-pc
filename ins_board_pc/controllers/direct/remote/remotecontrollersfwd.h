#ifndef REMOTECONTROLLERSFWD_H
#define REMOTECONTROLLERSFWD_H

#include "views/base/IBaseView.h"
#include "controllers/direct/remote/remotecontroller.h"

using PositionRemoteController = RemoteController<IPositionView>;
using OrientationRemoteController = RemoteController<IOrientationView>;

#endif // REMOTECONTROLLERSFWD_H
