//#include "mainwindow.h"

#include "views/enupositionview.h"
#include "views/xdorientationview.h"
#include "views/rpyorientationview.h"
#include "controllers/direct/filtering/filteringcontrollersfwd.h"
#include "controllers/direct/attributes/kalmanorientationattrcontroller.h"
#include "controllers/direct/attributes/kalmanpositionattrcontroller.h"
#include "controllers/switches/mixedmodelswitch.h"
#include "controllers/switches/positionmodelswitch.h"
#include "controllers/switches/orientationmodelswitch.h"
#include "controllers/metacontroller.h"

#include <QLineEdit>
#include <QComboBox>

#include "receiver.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    //MainWindow w;
    //w.show();

    ENUPositionView pos_view;
    XDOrientationView ori_xd;
    RPYOrientationView rpy_ori;

    QPushButton new_btn("Dummy");
    Receiver recv;

    QComboBox cb;

    PositionFilteringController pos_ctrl(&new_btn, &recv);
    pos_ctrl.attach_view(pos_view);

    OrientationFilteringController ori_ctrl(&new_btn, &recv);
    ori_ctrl.attach_view(ori_xd);
    ori_ctrl.attach_view(rpy_ori);

    QLineEdit le;
    KalmanOrientationAttrController attr_ctrl(&le,&le,&le,&le,&le,&le,&le,&le,&le);
    KalmanPositionAttrController pos_attr_ctrl(&le,&le,&le,&le,&le,&le);


    PositionModelSwitch pos_sw(&cb, pos_ctrl, pos_attr_ctrl);

    OrientationModelSwitch ori_sw(&cb, ori_ctrl, attr_ctrl);

    MixedModelSwitch mix_sw(&cb, pos_sw, ori_sw);

    MetaController meta_ctrl(&cb, pos_sw, ori_sw, mix_sw, pos_ctrl);

    meta_ctrl.configure_control(0);
    meta_ctrl.configure_control(1);

    return a.exec();
}
