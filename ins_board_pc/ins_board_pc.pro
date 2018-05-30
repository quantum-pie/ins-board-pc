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

INCLUDEPATH += D:/Qt5.8/Tools/mingw530_32/include += D:/tools/eigen += D:/tools/boost_1_62_0

SOURCES += filtering/filters/orientationcomplement.cpp \
    filtering/filters/positionbypass.cpp \
    filtering/filters/positionsim.cpp \
    filtering/private_implementation/kalmanorientationfilterbase.cpp \
    filtering/private_implementation/kalmanpositionfilterbase.cpp \
    filtering/private_implementation/mixedkalmanfilterbase.cpp \
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
    wmm/GeomagnetismLibrary.c \
    controllers/direct/attributes/kalmanorientationattrcontroller.cpp \
    controllers/direct/attributes/kalmanpositionattrcontroller.cpp \
    controllers/direct/attributes/simpositionattrcontroller.cpp \
    controllers/switches/modelswitchbase.cpp \
    controllers/switches/positionmodelswitch.cpp \
    controllers/switches/mixedmodelswitch.cpp \
    controllers/metacontroller.cpp \
    controllers/direct/filtering/filteringcontrollercommon.cpp \
    controllers/switches/orientationmodelswitch.cpp \
    views/orientation/rpyorientationview.cpp \
    views/orientation/stdorientationview.cpp \
    views/orientation/xdorientationview.cpp \
    views/position/enupositionview.cpp \
    views/position/trackpositionview.cpp \
    controllers/direct/rawcontroller.cpp \
    views/raw/xdrawmagnview.cpp \
    views/raw/rawaccelview.cpp \
    views/raw/rawgpsview.cpp \
    views/raw/rawgyroview.cpp \
    views/raw/rawmagnview.cpp \
    controllers/direct/attributes/complorientationattrcontroller.cpp \
    controllers/direct/magncalibrationcontroller.cpp \
    views/calibrationview.cpp

HEADERS  += \
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
    filtering/public_interfaces/IKalmanOrientationFilter.h \
    filtering/public_interfaces/IKalmanPositionFilter.h \
    filtering/public_interfaces/IOrientationFilter.h \
    filtering/public_interfaces/IPositionFilter.h \
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
    filtering/public_interfaces/ISimPositionFilter.h \
    core/IComplementOrientationAttr.h \
    core/IFilter.h \
    core/IKalmanOrientationAttr.h \
    core/IKalmanPositionAttr.h \
    core/IOrientationProvider.h \
    core/IPositionProvider.h \
    core/ISimPositionAttr.h \
    receiver.h \
    views/IBaseView.h \
    controllers/metacontroller.h \
    controllers/direct/controllerbase.h \
    controllers/direct/attributes/attrcontrollerbase.h \
    controllers/direct/attributes/kalmanorientationattrcontroller.h \
    controllers/direct/attributes/kalmanpositionattrcontroller.h \
    controllers/direct/attributes/simpositionattrcontroller.h \
    controllers/direct/filtering/filteringcontroller.h \
    controllers/direct/filtering/filteringcontrollercommon.h \
    controllers/switches/mixedmodelswitch.h \
    controllers/switches/orientationmodelswitch.h \
    controllers/switches/positionmodelswitch.h \
    controllers/switches/singlemodelswitchbase.h \
    controllers/switches/modelswitchbase.h \
    filtering/filters/filtersfwd.h \
    controllers/direct/filtering/filteringcontrollersfwd.h \
    controllers/direct/observablebase.h \
    controllers/direct/rawcontroller.h \
    views/orientation/rpyorientationview.h \
    views/orientation/stdorientationview.h \
    views/orientation/xdorientationview.h \
    views/position/enupositionview.h \
    views/raw/rawaccelview.h \
    views/raw/rawgpsview.h \
    views/raw/rawgyroview.h \
    views/raw/rawmagnview.h \
    views/position/trackpositionview.h \
    views/raw/xdrawmagnview.h \
    views/xdaxisplotview.h \
    controllers/direct/attributes/complorientationattrcontroller.h \
    controllers/direct/magncalibrationcontroller.h \
    views/calibrationview.h

FORMS    += mainwindow.ui
