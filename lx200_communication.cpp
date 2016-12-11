#include "lx200_communication.h"
#include <QDebug>

//--------------------------------------------------------

lx200_communication::lx200_communication(void) {

    rs232port.setPortName("/dev/ttyS0");
    rs232port.setBaudRate(QSerialPort::Baud9600);
    rs232port.setDataBits(QSerialPort::Data8);
    rs232port.setParity(QSerialPort::NoParity);
    rs232port.setStopBits(QSerialPort::OneStop);
    rs232port.setFlowControl(QSerialPort::SoftwareControl);
    if (!rs232port.open(QIODevice::ReadWrite)) {
        portIsUp = 0;
    } else {
        portIsUp = 1;
        rs232port.clear(QSerialPort::AllDirections);
    }
}

//--------------------------------------------------------

lx200_communication::~lx200_communication(void) {
    portIsUp = 0;
    rs232port.close();
}

//--------------------------------------------------------

bool lx200_communication::getPortState(void) {
    return portIsUp;
}

//--------------------------------------------------------

qint64 lx200_communication::getDataFromSerialPort(void) {
    qint64 charsToBeRead, charsRead=0;

    charsToBeRead=rs232port.bytesAvailable();
    if (charsToBeRead > 0) {
        if (rs232port.canReadLine() == true) {
            charsRead=rs232port.readLine(dataRead,4096);
        }
    }
    return charsRead;
}

//--------------------------------------------------------
