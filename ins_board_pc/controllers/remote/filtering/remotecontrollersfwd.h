#ifndef REMOTECONTROLLERSFWD_H
#define REMOTECONTROLLERSFWD_H

#include "views/base/IBaseView.h"
#include "controllers/remote/filtering/remotecontroller.h"

//! Remote position controller alias.
using PositionRemoteController = RemoteController<IPositionView>;

//! Remote orientation controller alias.
using OrientationRemoteController = RemoteController<IOrientationView>;

#endif // REMOTECONTROLLERSFWD_H
