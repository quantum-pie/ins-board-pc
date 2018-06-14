#ifndef REMOTECONTROLLERSFWD_H
#define REMOTECONTROLLERSFWD_H

#include "views/base/IBaseView.h"
#include "controllers/remote/filtering/remotecontroller.h"

using PositionRemoteController = RemoteController<IPositionView>;
using OrientationRemoteController = RemoteController<IOrientationView>;

#endif // REMOTECONTROLLERSFWD_H
