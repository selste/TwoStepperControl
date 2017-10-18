#-------------------------------------------------
#
# Project created by QtCreator 2017-10-16T14:50:07
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = SPI_MotorMaster
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    spi_drive.cpp

HEADERS  += mainwindow.h \
    spi_drive.h

FORMS    += mainwindow.ui

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../usr/lib/release/ -lwiringPi
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../usr/lib/debug/ -lwiringPi
else:unix: LIBS += -L$$PWD/../../../usr/lib/ -lwiringPi

#INCLUDEPATH += $$PWD/../../../usr/include
#DEPENDPATH += $$PWD/../../../usr/include
