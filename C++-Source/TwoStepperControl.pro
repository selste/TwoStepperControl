#-------------------------------------------------
#
# Project created by QtCreator 2016-10-30T19:59:37
#
#-------------------------------------------------

QT       += core gui
QT       += serialport
QT       += network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = TwoStepperControl
TEMPLATE = app

CONFIG += c++11
CONFIG += j4

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    currentObjectCatalog.cpp \
    QDisplay2D.cpp \
    tsc_globaldata.cpp \
    lx200_communication.cpp \
    ocv_guiding.cpp \
    ccd_client.cpp \
    QtKineticStepper.cpp \
    QtContinuousStepper.cpp \
    tsc_bt_serialcomm.cpp \
    spi_drive.cpp

HEADERS  += \
    mainwindow.h \
    currentObjectCatalog.h \
    QDisplay2D.h \
    tsc_globaldata.h \
    lx200_communication.h \
    ocv_guiding.h \
    ccd_client.h \
    QtKineticStepper.h \
    QtContinuousStepper.h \
    tsc_bt_serialcomm.h \
    spi_drive.h

# INCLUDEPATH += /home/pi
# INCLUDEPATH += /home/pi/libindi/libs/
# INCLUDEPATH += /home/pi/libindi/
INCLUDEPATH += /usr/local/include/opencv2
INCLUDEPATH += /usr/include/libindi

FORMS    += mainwindow.ui

QMAKE_DEFAULT_INCDIRS = \\

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lphidget21
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lphidget21
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lphidget21



win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../usr/lib/release/ -lindiclient
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../usr/lib/debug/ -lindiclient
else:unix: LIBS += -L$$PWD/../../../../usr/lib/ -lindiclient


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../usr/lib/arm-linux-gnueabihf/release/ -lm
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../usr/lib/arm-linux-gnueabihf/debug/ -lm
else:unix: LIBS += -L$$PWD/../../../../usr/lib/arm-linux-gnueabihf/ -lm

INCLUDEPATH += $$PWD/../../../../usr/lib/arm-linux-gnueabihf
DEPENDPATH += $$PWD/../../../../usr/lib/arm-linux-gnueabihf

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../usr/lib/arm-linux-gnueabihf/release/ -lz
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../usr/lib/arm-linux-gnueabihf/debug/ -lz
else:unix: LIBS += -L$$PWD/../../../../usr/lib/arm-linux-gnueabihf/ -lz

INCLUDEPATH += $$PWD/../../../../usr/lib/arm-linux-gnueabihf
DEPENDPATH += $$PWD/../../../../usr/lib/arm-linux-gnueabihf

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../usr/lib/arm-linux-gnueabihf/release/ -lcfitsio
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../usr/lib/arm-linux-gnueabihf/debug/ -lcfitsio
else:unix: LIBS += -L$$PWD/../../../../usr/lib/arm-linux-gnueabihf/ -lcfitsio

INCLUDEPATH += $$PWD/../../../../usr/lib/arm-linux-gnueabihf
DEPENDPATH += $$PWD/../../../../usr/lib/arm-linux-gnueabihf

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../usr/lib/arm-linux-gnueabihf/release/ -lpthread
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../usr/lib/arm-linux-gnueabihf/debug/ -lpthread
else:unix: LIBS += -L$$PWD/../../../../usr/lib/arm-linux-gnueabihf/ -lpthread

INCLUDEPATH += $$PWD/../../../../usr/lib/arm-linux-gnueabihf
DEPENDPATH += $$PWD/../../../../usr/lib/arm-linux-gnueabihf

unix:!macx: LIBS += -L$$PWD/../../../usr/local/lib/ -lopencv_core

INCLUDEPATH += $$PWD/../../../usr/local/include
DEPENDPATH += $$PWD/../../../usr/local/include

unix:!macx: LIBS += -L$$PWD/../../../usr/local/lib/ -lopencv_imgproc

INCLUDEPATH += $$PWD/../../../usr/local/include
DEPENDPATH += $$PWD/../../../usr/local/include

unix:!macx: LIBS += -L$$PWD/../../../usr/lib/ -lwiringPi

INCLUDEPATH += $$PWD/../../../usr/include
DEPENDPATH += $$PWD/../../../usr/include

unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/arm-linux-gnueabihf/ -lnova

INCLUDEPATH += $$PWD/../../../../usr/lib/arm-linux-gnueabihf
DEPENDPATH += $$PWD/../../../../usr/lib/arm-linux-gnueabihf
