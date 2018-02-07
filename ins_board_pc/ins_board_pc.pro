#-------------------------------------------------
#
# Project created by QtCreator 2017-12-28T12:24:10
#
#-------------------------------------------------

QT       += core gui network 3dcore 3drender 3dinput 3dlogic 3dextras

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport datavisualization

TARGET = ins_board_pc
TEMPLATE = app

CONFIG += c++14 #console
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

INCLUDEPATH += D:/Qt5.8/Tools/mingw530_32/include

SOURCES += main.cpp\
        mainwindow.cpp \
    qcustomplot.cpp \
    calibrator.cpp \
    quatkalman.cpp \
    wmm/GeomagnetismLibrary.c \
    wmmwrapper.cpp \
    qualitycontrol.cpp \
    quatcomplement.cpp \
    quaternions.cpp \
    ublasaux.cpp

HEADERS  += mainwindow.h \
    qcustomplot.h \
    calibrator.h \
    quatkalman.h \
    wmm/GeomagnetismHeader.h \
    wmmwrapper.h \
    qualitycontrol.h \
    quatcomplement.h \
    quaternions.h \
    ublasaux.h \
    physconst.h

FORMS    += mainwindow.ui
