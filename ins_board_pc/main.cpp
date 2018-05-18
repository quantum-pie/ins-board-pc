//#include "mainwindow.h"
#include "models/kalmanpositionfilteringmodel.h"
#include "controllers/kalmanpositionfilteringcontroller.h"
#include "filtering/filters/generickalman.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    //MainWindow w;
    //w.show();

    FullEKF ekf_flt;
    PositionEKF ekfpos;

    KalmanPositionFilteringModel model(&ekf_flt);
    KalmanPositionFilteringController ctrl(model);
    ctrl.handle_kp_strategy(&ekfpos);


    return a.exec();
}
