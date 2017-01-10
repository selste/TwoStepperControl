#-------------------------------------------------
#
# Project created by QtCreator 2016-10-30T19:59:37
#
#-------------------------------------------------

QT       += core gui
QT       += serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = TwoStepperControl
TEMPLATE = app

CONFIG += c++11


SOURCES += \
    main.cpp \
    mainwindow.cpp \
    alccd5_client.cpp \
    currentObjectCatalog.cpp \
    QDisplay2D.cpp \
    tsc_globaldata.cpp \
    qstepperphidgetsDecl.cpp \
    qstepperphidgetsRA.cpp \
    lx200_communication.cpp \
    ocv_guiding.cpp

HEADERS  += \
    mainwindow.h \
    alccd5_client.h \
    currentObjectCatalog.h \
    QDisplay2D.h \
    tsc_globaldata.h \
    qstepperphidgetsDecl.h \
    qstepperphidgetsRA.h \
    lx200_communication.h \
    ocv_guiding.h

INCLUDEPATH += /home/pi
INCLUDEPATH += /home/pi/libindi/libs/
INCLUDEPATH += /home/pi/libindi/
INCLUDEPATH += /usr/local/include/opencv2

FORMS    += mainwindow.ui

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lphidget21
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lphidget21
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lphidget21

INCLUDEPATH += $$PWD/../../../usr/include
DEPENDPATH += $$PWD/../../../usr/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../usr/lib/release/ -lindiclient
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../usr/lib/debug/ -lindiclient
else:unix: LIBS += -L$$PWD/../../../../usr/lib/ -lindiclient

INCLUDEPATH += $$PWD/../../../../usr/include
DEPENDPATH += $$PWD/../../../../usr/include

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../../usr/lib/release/libindiclient.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../../usr/lib/debug/libindiclient.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../../usr/lib/release/indiclient.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../../usr/lib/debug/indiclient.lib
else:unix: PRE_TARGETDEPS += $$PWD/../../../../usr/lib/libindiclient.a

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../usr/lib/release/ -lindi
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../usr/lib/debug/ -lindi
else:unix: LIBS += -L$$PWD/../../../../usr/lib/ -lindi

INCLUDEPATH += $$PWD/../../../../usr/include
DEPENDPATH += $$PWD/../../../../usr/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../usr/lib/arm-linux-gnueabihf/release/ -lnova
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../usr/lib/arm-linux-gnueabihf/debug/ -lnova
else:unix: LIBS += -L$$PWD/../../../../usr/lib/arm-linux-gnueabihf/ -lnova

INCLUDEPATH += $$PWD/../../../../usr/lib/arm-linux-gnueabihf
DEPENDPATH += $$PWD/../../../../usr/lib/arm-linux-gnueabihf

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


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../opencv/opencv/opencv_build/lib/release/ -lopencv_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../opencv/opencv/opencv_build/lib/debug/ -lopencv_core
else:unix: LIBS += -L$$PWD/../opencv/opencv/opencv_build/lib/ -lopencv_core

INCLUDEPATH += $$PWD/../opencv/opencv/opencv_build/include
DEPENDPATH += $$PWD/../opencv/opencv/opencv_build/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/local/lib/release/ -lopencv_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/local/lib/debug/ -lopencv_core
else:unix: LIBS += -L$$PWD/../../../usr/local/lib/ -lopencv_core

INCLUDEPATH += $$PWD/../../../usr/local/include
DEPENDPATH += $$PWD/../../../usr/local/include
