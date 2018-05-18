#-------------------------------------------------
#
# Project created by QtCreator 2017-12-28T12:24:10
#
#-------------------------------------------------

QT       += core gui network 3dcore 3drender 3dinput 3dlogic 3dextras

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport datavisualization

TARGET = ins_board_pc
TEMPLATE = app

OBJECTS_DIR = obj
MOC_DIR = moc

CONFIG += c++14 #console
#CONFIG -= app_bundle

QMAKE_CXXFLAGS += -DNDEBUG

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += C:/Qt/Tools/mingw530_32/include += C:/Tools/Eigen3/include/eigen3 += C:/Tools/boost_1_66_0

SOURCES += filtering/filters/orientationcomplement.cpp \
    filtering/filters/positionbypass.cpp \
    filtering/filters/positionsim.cpp \
    filtering/private_implementation/kalmanorientationfilterbase.cpp \
    filtering/private_implementation/kalmanpositionfilterbase.cpp \
    filtering/private_implementation/mixedkalmanfilterbase.cpp \
    models/complorientationfilteringmodel.cpp \
    models/kalmanorientationfilteringmodel.cpp \
    models/kalmanpositionfilteringmodel.cpp \
    models/orientationfilteringmodel.cpp \
    models/positionfilteringmodel.cpp \
    models/simpositionfilteringmodel.cpp \
    views/enupositionview.cpp \
    views/rpyorientationview.cpp \
    views/xdorientationview.cpp \
    earth.cpp \
    eigenaux.cpp \
    ellipsoid.cpp \
    geometry.cpp \
    gravity.cpp \
    horizon.cpp \
    magncalibrator.cpp \
    magnetic.cpp \
    main.cpp \
    mainwindow.cpp \
    packets.cpp \
    qcustomplot.cpp \
    quaternion.cpp \
    quatutils.cpp \
    utils.cpp \
    wmm/GeomagnetismLibrary.c

HEADERS  += controllers/kalmanorientationfilteringcontroller.h \
    controllers/kalmanpositionfilteringcontroller.h \
    filtering/filters/generickalman.h \
    filtering/filters/orientationcomplement.h \
    filtering/filters/positionbypass.h \
    filtering/filters/positionsim.h \
    filtering/plugins/ekfcorrector.h \
    filtering/plugins/filterplugins.h \
    filtering/plugins/kfextrapolator.h \
    filtering/plugins/ukfcorrector.h \
    filtering/private_implementation/IFilterBase.h \
    filtering/private_implementation/kalmanorientationfilterbase.h \
    filtering/private_implementation/kalmanpositionfilterbase.h \
    filtering/private_implementation/mixedkalmanfilterbase.h \
    filtering/public_interfaces/IComplementOrientationFilter.h \
    filtering/public_interfaces/IFilter.h \
    filtering/public_interfaces/IKalmanOrientationFilter.h \
    filtering/public_interfaces/IKalmanPositionFilter.h \
    filtering/public_interfaces/IOrientationFilter.h \
    filtering/public_interfaces/IPositionFilter.h \
    models/complorientationfilteringmodel.h \
    models/kalmanorientationfilteringmodel.h \
    models/kalmanpositionfilteringmodel.h \
    models/orientationfilteringmodel.h \
    models/positionfilteringmodel.h \
    models/simpositionfilteringmodel.h \
    models/basefilteringmodel.h \
    views/enupositionview.h \
    views/rpyorientationview.h \
    views/xdorientationview.h \
    wmm/GeomagnetismHeader.h \
    earth.h \
    eigenaux.h \
    ellipsoid.h \
    geometry.h \
    gravity.h \
    horizon.h \
    magncalibrator.h \
    magnetic.h \
    mainwindow.h \
    packets.h \
    qcustomplot.h \
    qualitycontrol.h \
    quaternion.h \
    quatfwd.h \
    quatutils.h \
    utils.h \
    controllers/basefilteringcontroller.h \
    controllers/kalmanfilteringmetacontroller.h \
    views/attrkalmanpositionview.h \
    views/attrkalmanorientationview.h

FORMS    += mainwindow.ui
