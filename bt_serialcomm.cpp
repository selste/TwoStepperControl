#include "bt_serialcomm.h"
#include <QDebug>

//---------------------------------------------------
bt_serialcomm::bt_serialcomm(void) {
    system("sudo rfcomm bind hci0 98:D3:31:FB:2A:8C");
    this->portIsUp=false;
    rfcommport.setPortName("/dev/rfcomm0");
    rfcommport.setBaudRate(QSerialPort::Baud9600);
    rfcommport.setDataBits(QSerialPort::Data8);
    rfcommport.setParity(QSerialPort::NoParity);
    rfcommport.setStopBits(QSerialPort::OneStop);
    rfcommport.setFlowControl(QSerialPort::NoFlowControl);
    this->incomingCommand = new QString();
}

//---------------------------------------------------
bt_serialcomm::~bt_serialcomm(void) {
    if (portIsUp == 1) {
        rfcommport.setBreakEnabled(true);
        rfcommport.clear(QSerialPort::AllDirections);
        rfcommport.close();
        portIsUp = 0;
    }
    delete incomingCommand;
}

//---------------------------------------------------
void bt_serialcomm::shutDownPort(void) {
    rfcommport.setBreakEnabled(true);
    portIsUp = 0;
    rfcommport.clear(QSerialPort::AllDirections);
    rfcommport.close();
}

//---------------------------------------------------
void bt_serialcomm::openPort(void) {
    portIsUp = 1;
    qDebug() << "Opening Port";
    if (!rfcommport.open(QIODevice::ReadWrite)) {
        portIsUp = 0;
    } else {
        rfcommport.setBreakEnabled(false);
        portIsUp = 1;
        rfcommport.clear(QSerialPort::AllDirections);
    }
    qDebug() << "Port is up:" << portIsUp;
}

//---------------------------------------------------
bool bt_serialcomm::getPortState(void) {
    return this->portIsUp;
}

//---------------------------------------------------
qint64 bt_serialcomm::getDataFromSerialPort(void) {
    char buf[1024];
    qint64 lineLength;

    if (rfcommport.canReadLine() == true) {
        lineLength = rfcommport.readLine(buf, sizeof(buf));
        if (lineLength != -1) {
            this->incomingCommand->clear();
            this->incomingCommand->append(buf);
            this->incomingCommand->chop(2);
            qDebug() << incomingCommand->toLatin1();
        }
    }
    return lineLength;
}

//---------------------------------------------------



