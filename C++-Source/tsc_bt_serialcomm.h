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

#ifndef TSC_BT_SERIALCOMM_H
#define TSC_BT_SERIALCOMM_H

#include <QObject>
#include <QString>
#include <QCoreApplication>
#include <QtSerialPort/qserialport.h>
#include <QtSerialPort/qserialportinfo.h>

class tsc_bt_serialcomm :public QObject {
    Q_OBJECT
public:
    tsc_bt_serialcomm(QString);
    void bt_serialcommTryRestart(QString);
    ~tsc_bt_serialcomm(void);
    void shutDownPort(void);
    void openPort(void);
    bool getPortState(void);
    qint64 getDataFromSerialPort(void);
    QString* getTSCcommand(void);

private:
    QSerialPort rfcommport;
    QString *incomingCommand;
    bool portIsUp;

signals:
    void btDataReceived();
};
#endif
// TSC_BT_SERIALCOMM_H
