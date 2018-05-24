//#include "mainwindow.h"
//#include "models/kalmanpositionfilteringmodel.h"
//#include "controllers/kalmanpositionfilteringcontroller.h"
#include "filtering/filters/generickalman.h"
#include "filtering/filters/positionsim.h"

#include "views/enupositionview.h"
#include "views/xdorientationview.h"
#include "views/rpyorientationview.h"
#include "controllers/filteringcontroller.h"
#include "controllers/kalmanorientationattrcontroller.h"
#include "controllers/simpositionattrcontroller.h"

#include <QLineEdit>

#include "receiver.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    //MainWindow w;
    //w.show();

    FullEKF ekf_flt;
    PositionEKF ekfpos;
    PositionSim pos_sim;

    ENUPositionView pos_view;
    XDOrientationView ori_xd;
    RPYOrientationView rpy_ori;

    QPushButton new_btn("Dummy");
    Receiver recv;

    PositionFilteringController pos_ctrl(&new_btn, &recv);
    pos_ctrl.set_model(&ekf_flt);
    pos_ctrl.attach_view(pos_view);

    OrientationFilteringController ori_ctrl(&new_btn, &recv);
    ori_ctrl.set_model(&ekf_flt);
    ori_ctrl.attach_view(ori_xd);

    QLineEdit le;
    KalmanOrientationAttrController attr_ctrl(&le,&le,&le,&le,&le,&le,&le,&le,&le);
    attr_ctrl.set_model(&ekf_flt);

    SimPositionAttrController sim_ctr(&le, &le);
    sim_ctr.set_model(&pos_sim);

    return a.exec();
}
