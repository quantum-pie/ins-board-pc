//#include "mainwindow.h"
#include <QApplication>

#include "mixedkalmanfilterbase.h"
#include "kfextrapolator.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    //MainWindow w;
    //w.show();

    KFExtrapolator<MixedKalmanFilterBase> cool;

    return a.exec();
}
