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
