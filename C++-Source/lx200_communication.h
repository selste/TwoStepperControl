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
// thi class implements the basic LX200 protocol of TSC

#ifndef LX200_COMMUNICATION_H
#define LX200_COMMUNICATION_H

#include <QTimer>
#include <QByteArray>
#include <QObject>
#include <QString>
#include <QCoreApplication>
#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QHostAddress>

class lx200_communication:public QObject {
    Q_OBJECT
public:
    lx200_communication(void);
    ~lx200_communication(void);
    void handleDataFromClient(QString);
    double getReceivedCoordinates(short); // 0 for RA, 1 for Decl - retrieve data conveyed from LX
    QString* getLX200Command(void);
    QString* getLX200Response(void);
    QString* getLX200ResponseRA(void);
    QString* getLX200ResponseDecl(void);
    void clearReplyString(void);
    void setNumberFormat(bool);

private:
    QString *replyStrLX;
    QString *assembledString;
    QString *msgRAString;
    QString *msgDeclString;
    QString *incomingCommand;
    QString *subCmd;
    QString *lastSubCmd;
    double receivedRAFromLX;
    double receivedDeclFromLX;
    bool handleBasicLX200Protocol(QString);
    bool gotRACoordinates;
    bool gotDeclCoordinates;
    bool sendSimpleCoordinates; // use either ddd.mm or ddd.mm.ss
    void assembleDeclinationString(void);
    void assembleRAString(void);
    void sendCommand(short what);

    struct LX200CommandStruct {
        QString slewRA;
        QString slewDecl;
        QString slewPossible;
        QString syncCommand;
        QString getDecl;
        QString getRA;
        QString getHiDef;
        QString moveEast;
        QString moveWest;
        QString moveNorth;
        QString moveSouth;
        QString stopMoveEast;
        QString stopMoveWest;
        QString stopMoveNorth;
        QString stopMoveSouth;
        QString setCenterSpeed;
        QString setGuideSpeed;
        QString setFindSpeed;
        QString setGOTOSpeed;
        QString stopMotion;
        QString getCalendarFormat;
        QString getName;
        QString getDate;
        QString getTrackingRate;
        QString getLatitude;
        QString getLongitude;
        QString getUTCOffset;
        QString getLocalTime;
        QString setLocalTime;
        QString setLongitude;
    };
    struct LX200CommandStruct LX200Commands;
signals:
    void RS232CommandReceived(void);
    void logRASent(void);
    void logDeclSent(void);
    void logCommandSent(void);
    void RS232slew(void);
    void RS232sync(void);
    void RS232stopMotion(void);
    void RS232stopMoveEast(void);
    void RS232stopMoveWest(void);
    void RS232stopMoveNorth(void);
    void RS232stopMoveSouth(void);
    void RS232moveEast(void);
    void RS232moveWest(void);
    void RS232moveNorth(void);
    void RS232moveSouth(void);
    void RS232centerSpeed(void);
    void RS232guideSpeed(void);
    void RS232findSpeed(void);
    void RS232gotoSpeed(void);
    void polarAlignmentSignal(void);
    void clientRASent(QString*);
    void clientDeclSent(QString*);
    void clientCommandSent(QString*);
};

#endif // LX200_COMMUNICATION_H
