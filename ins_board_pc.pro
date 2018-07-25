#-------------------------------------------------
#
# Project created by QtCreator 2017-12-28T12:24:10
#
#-------------------------------------------------

QT       += core gui network 3dcore 3drender 3dinput 3dlogic 3dextras testlib

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

INCLUDEPATH += $$(EIGEN_INCLUDE) += $$(BOOST_INCLUDE)

SOURCES += filtering/filters/orientationcomplement.cpp \
    filtering/filters/positionbypass.cpp \
    filtering/filters/positionsim.cpp \
    filtering/private_implementation/kalmanorientationfilterbase.cpp \
    filtering/private_implementation/kalmanpositionfilterbase.cpp \
    filtering/private_implementation/mixedkalmanfilterbase.cpp \
    earth.cpp \
    ellipsoid.cpp \
    geometry.cpp \
    gravity.cpp \
    horizon.cpp \
    magncalibrator.cpp \
    magnetic.cpp \
    main.cpp \
    mainwindow.cpp \
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
    views/raw/xdrawmagnview.cpp \
    views/raw/rawaccelview.cpp \
    views/raw/rawgpsview.cpp \
    views/raw/rawgyroview.cpp \
    views/raw/rawmagnview.cpp \
    controllers/direct/attributes/complorientationattrcontroller.cpp \
    views/calibration/xdcalibrationview.cpp \
    views/calibration/numcalibrationview.cpp \
    filtering/plugins/utparams.cpp \
    views/base/xdaxisscatter.cpp \
    views/base/plotsetup.cpp \
    adapters/positionfilteringviewmodel.cpp \
    adapters/orientationfilteringviewmodel.cpp \
    controllers/remote/attributes/remotecomploriattr.cpp \
    controllers/remote/attributes/remotekalmanposattr.cpp \
    communication/receiver.cpp \
    communication/terminal.cpp \
    communication/terminalbase.cpp \
    communication/udpreceiver.cpp \
    communication/udpsender.cpp \
    controllers/direct/magncalibrationcontroller.cpp \
    controllers/direct/rawcontroller.cpp

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
    controllers/metacontroller.h \
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
    controllers/direct/attributes/complorientationattrcontroller.h \
    controllers/direct/magncalibrationcontroller.h \
    views/calibration/xdcalibrationview.h \
    views/calibration/numcalibrationview.h \
    filtering/plugins/utparams.h \
    views/base/IBaseView.h \
    views/base/xdaxisscatter.h \
    views/base/plotsetup.h \
    adapters/adapter.h \
    adapters/positionfilteringviewmodel.h \
    adapters/orientationfilteringviewmodel.h \
    controllers/controllerbase.h \
    controllers/observablebase.h \
    controllers/runningflag.h \
    controllers/remote/attributes/remotecomploriattr.h \
    controllers/remote/attributes/remotekalmanposattr.h \
    controllers/remote/filtering/remotebase.h \
    controllers/remote/filtering/remotecontroller.h \
    controllers/remote/filtering/remotecontrollersfwd.h \
    communication/receiver.h \
    communication/terminal.h \
    communication/terminalbase.h \
    communication/udpreceiver.h \
    communication/udpsender.h \
    controllers/direct/rawcontroller.h \
    communication/var.h

FORMS    += mainwindow.ui
