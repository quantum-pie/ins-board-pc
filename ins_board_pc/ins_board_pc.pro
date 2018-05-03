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

CONFIG += c++17 #console
#CONFIG -= app_bundle

QMAKE_CXXFLAGS += -DNDEBUG -DBOOST_UBLAS_NDEBUG

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += D:/Qt5.8/Tools/mingw530_32/include += D:/tools/eigen

SOURCES += main.cpp\
        mainwindow.cpp \
    qcustomplot.cpp \
    calibrator.cpp \
    wmm/GeomagnetismLibrary.c \
    wmmwrapper.cpp \
    qualitycontrol.cpp \
    quaternions.cpp \
    eigenaux.cpp \
    fullekf.cpp \
    fullukf.cpp \
    positionlkf.cpp \
    orientationcomplement.cpp \
    orientationekf.cpp \
    earth.cpp \
    ellipsoid.cpp \
    geometry.cpp \
    gravity.cpp \
    magncalibrator.cpp \
    magnetic.cpp \
    quaternion.cpp \
    positionbypass.cpp \
    positionsim.cpp

HEADERS  += mainwindow.h \
    qcustomplot.h \
    calibrator.h \
    wmm/GeomagnetismHeader.h \
    wmmwrapper.h \
    qualitycontrol.h \
    physconst.h \
    eigenaux.h \
    positionfilter.h \
    orientationfilter.h \
    kalmanpositionfilter.h \
    kalmanorientationfilter.h \
    positionlkf.h \
    orientationcomplement.h \
    orientationekf.h \
    fullukf.h \
    fullekf.h \
    earth.h \
    ellipsoid.h \
    geometry.h \
    gravity.h \
    magncalibrator.h \
    magnetic.h \
    quaternion.h \
    IComplementOrientationFilter.h \
    IFilter.h \
    IKalmanOrientationFilter.h \
    IKalmanPositionFilter.h \
    IOrientationFilter.h \
    IPositionFilter.h \
    positionbypass.h \
    positionsim.h

FORMS    += mainwindow.ui
