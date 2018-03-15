// this code is part of "TSC", a free control software for astronomical telescopes
// Copyright (C)  2016-18, wolfgang birkfellner
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

//---------------------------------------------------
// this class handles serial bluetooth communications for the BT-handbox

#include "bt_serialcomm.h"
#include <QDebug>

//---------------------------------------------------
// constructor, starts up a standard serial port for the
// MAC address of the handbox ...

bt_serialcomm::bt_serialcomm(QString bt_MACaddr) {
    QString startupRFPort;
    this->portIsUp=false;

    startupRFPort.append("sudo rfcomm connect hci0 ");
    startupRFPort.append(bt_MACaddr);
    startupRFPort.append(" &");
    system(startupRFPort.toLatin1());
    rfcommport.setPortName("/dev/rfcomm0");
    rfcommport.setBaudRate(QSerialPort::Baud9600);
    rfcommport.setDataBits(QSerialPort::Data8);
    rfcommport.setParity(QSerialPort::NoParity);
    rfcommport.setStopBits(QSerialPort::OneStop);
    rfcommport.setFlowControl(QSerialPort::NoFlowControl);
    this->incomingCommand = new QString();
}

//---------------------------------------------------
// tries to re-open a serial port
void bt_serialcomm::bt_serialcommTryRestart(QString bt_MACaddr) {
    QString startupRFPort;
    this->portIsUp=false;

    startupRFPort.append("sudo rfcomm connect hci0 ");
    startupRFPort.append(bt_MACaddr);
    startupRFPort.append(" &");
    system(startupRFPort.toLatin1());
    rfcommport.setPortName("/dev/rfcomm0");
    rfcommport.setBaudRate(QSerialPort::Baud9600);
    rfcommport.setDataBits(QSerialPort::Data8);
    rfcommport.setParity(QSerialPort::NoParity);
    rfcommport.setStopBits(QSerialPort::OneStop);
    rfcommport.setFlowControl(QSerialPort::NoFlowControl);
}

//----------------------------------------------------
// desctructor; closes the serial port
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
// closes the serial port
void bt_serialcomm::shutDownPort(void) {
    rfcommport.setBreakEnabled(true);
    portIsUp = 0;
    rfcommport.clear(QSerialPort::AllDirections);
    rfcommport.close();
}

//---------------------------------------------------
// opens a serial port for BT-communicaition
void bt_serialcomm::openPort(void) {
    portIsUp = 1;
    if (!rfcommport.open(QIODevice::ReadWrite)) {
        portIsUp = 0;
    } else {
        rfcommport.setBreakEnabled(false);
        portIsUp = 1;
        rfcommport.clear(QSerialPort::AllDirections);
    }
}

//---------------------------------------------------
// returns the state of the port
bool bt_serialcomm::getPortState(void) {
    return this->portIsUp;
}

//---------------------------------------------------
// reads max 1064 byte from the serial port and emits a signal that data are available
qint64 bt_serialcomm::getDataFromSerialPort(void) {
    char buf[1024];
    qint64 lineLength=0;

    if (rfcommport.bytesAvailable() > 0) {
        if (rfcommport.canReadLine() == true) {
            lineLength = rfcommport.readLine(buf, sizeof(buf));
            if (lineLength != -1) {
                this->incomingCommand->clear();
                this->incomingCommand->append(buf);
                this->incomingCommand->chop(2);
                rfcommport.clear(QSerialPort::AllDirections);
                emit this->btDataReceived();
            }
        }
    }
    return lineLength;
}

//---------------------------------------------------
// returns the received string
QString* bt_serialcomm::getTSCcommand(void) {
    return this->incomingCommand;
}

