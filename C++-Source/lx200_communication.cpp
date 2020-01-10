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

#include "lx200_communication.h"
#include "tsc_globaldata.h"
#include <QDebug>
#include <unistd.h>
#include <math.h>

extern TSC_GlobalData *g_AllData;

//--------------------------------------------------------

lx200_communication::lx200_communication(void) {
    replyStrLX = new QString();
    assembledString = new QString();
    this->incomingCommand = new QString();
    this->msgRAString = new QString();
    this->msgDeclString = new QString();
    this->subCmd = new QString();
    this->lastSubCmd = new QString(); // a buffer holding the last command, just for protocols
    this->timeString = new QString();
    this->dateString = new QString();
    this->lutc = 0;
    this->llong = 15;
    this->llat = 48;
    receivedRAFromLX = 0.0;
    receivedDeclFromLX = 0.0;
    gotRACoordinates = false;
    gotDeclCoordinates = false;
    sendSimpleCoordinates=false; // determine the reponse format as ddd:mm or ddd:mm:ss
    gotDate = false;
    gotTime = false;
    gotLat = false;
    gotLong = false;
    gotUTCOffset = false;
    LX200Commands.getDecl = QString("GD");
    LX200Commands.getRA = QString("GR");
    LX200Commands.getHiDef = QString("U");
    LX200Commands.stopMotion = QString("Q");
    LX200Commands.slewRA = QString("Sr");
    LX200Commands.slewDecl = QString("Sd");
    LX200Commands.slewPossible = QString("MS");
    LX200Commands.syncCommand = QString("CM");
    LX200Commands.moveEast = QString("Me");
    LX200Commands.moveWest = QString("Mw");
    LX200Commands.moveNorth = QString("Mn");
    LX200Commands.moveSouth = QString("Ms");
    LX200Commands.stopMoveEast = QString("Qe");
    LX200Commands.stopMoveWest = QString("Qw");
    LX200Commands.stopMoveNorth = QString("Qn");
    LX200Commands.stopMoveSouth = QString("Qs");
    LX200Commands.setCenterSpeed = QString("RC");
    LX200Commands.setGuideSpeed = QString("RG");
    LX200Commands.setFindSpeed = QString("RM");
    LX200Commands.setGOTOSpeed = QString("RS");
    LX200Commands.getCalendarFormat  = QString("Gc");
    LX200Commands.getDate = QString("GC");
    LX200Commands.getName = QString("GM");
    LX200Commands.getTrackingRate = QString("GT");
    LX200Commands.getLatitude = QString("Gt");
    LX200Commands.getLongitude = QString("Gg");
    LX200Commands.getUTCOffset = QString("GG");
    LX200Commands.setUTCOffset = QString("SG");
    LX200Commands.getLocalTime = QString("GL");
    LX200Commands.setLocalTime = QString("SL");
    LX200Commands.setLocalDate = QString("SC");
    LX200Commands.setLongitude = QString("Sg");
    LX200Commands.setLatitude = QString("St");
}

//--------------------------------------------------------

lx200_communication::~lx200_communication(void) {
    delete replyStrLX;
    delete incomingCommand;
    delete assembledString;
    delete msgRAString;
    delete msgDeclString;
    delete subCmd;
    delete lastSubCmd;
    delete timeString;
    delete dateString;
}

//--------------------------------------------------------
void lx200_communication::clearReplyString(void) {
    this->replyStrLX->clear();
}

//--------------------------------------------------------

void lx200_communication::handleDataFromClient(QString cmdData) {
    QStringList *subCmdList;
    QString chopSCMD;
    int cmdCounter;
    long scmdLen;

    this->incomingCommand->append(cmdData);
        // first, take care of the LX 200 classic protocol which establishes communication by
        // sending an <ACK> and receiving a character on the mount type.
    if ((int)((this->incomingCommand->toLatin1())[0])==6) {
        emit this->polarAlignmentSignal(); // this signal triggers sending "P" ....
        return;
    }
    this->lastSubCmd->clear();
    this->lastSubCmd->append(this->incomingCommand->toLatin1());
//    if (this->incomingCommand->length() > 0) {
//        qDebug() << "Received: " << this->incomingCommand->toLatin1();
//    }

    subCmdList = new QStringList(this->incomingCommand->split("#:", QString::SkipEmptyParts));
    if (subCmdList->isEmpty()==false) {
        for (cmdCounter = 0; cmdCounter < (subCmdList->length()-1); cmdCounter++) {
            this->subCmd->append(subCmdList->at(cmdCounter));
            this->subCmd->remove("#");
            if (this->subCmd->startsWith(":",Qt::CaseSensitive)==true) {
                scmdLen=this->subCmd->length();
                chopSCMD=this->subCmd->right((scmdLen-1));
                this->subCmd->clear();
                this->subCmd->append(chopSCMD);
            } // some people send commands starting with two ":" - don't ask me why ...
                emit this->RS232CommandReceived();
                this->handleBasicLX200Protocol(*subCmd);
                this->subCmd->clear();
        }
        this->subCmd->append(subCmdList->at(subCmdList->length()-1));
        delete subCmdList;
        this->incomingCommand->clear();
        if (this->subCmd->right(1)!="#") {
            this->incomingCommand->append("#:");
            this->incomingCommand->append(this->subCmd); // remember the command until next time; should only be necessary with serial commands
        } else {
            this->subCmd->remove("#:");
            this->subCmd->remove("#");
            if (this->subCmd->startsWith(":",Qt::CaseSensitive)==true) {
                scmdLen=this->subCmd->length();
                chopSCMD=this->subCmd->right((scmdLen-1));
                this->subCmd->clear();
                this->subCmd->append(chopSCMD);
            }
            emit this->RS232CommandReceived();
            this->handleBasicLX200Protocol(*subCmd);
            this->subCmd->clear();
        }
    }
}

//--------------------------------------------------------

bool lx200_communication::handleBasicLX200Protocol(QString cmd) {
    QString *lx200cmd, *numSubStr;
    QString *helper;
    QStringList commandList,numericalList;
    int numberOfCommands, cmdCounter,toDegs, coordMins,coordSign;
    double rah,ram,ras,decldeg,declmin,declsec, toMins, coordDeg,
            coordMin, lhh, lmm, lss, lmonth, lday, lyear;
    short declSign;
    QElapsedTimer *waitTimer;

    helper = new QString();
    lx200cmd = new QString();
    numSubStr = new QString();
    commandList = cmd.split('#',QString::SkipEmptyParts,Qt::CaseSensitive);
    numberOfCommands=commandList.count();

    for (cmdCounter = 0; cmdCounter < numberOfCommands; cmdCounter++) {
        lx200cmd->clear();
        assembledString->clear();
        lx200cmd->append(commandList[cmdCounter]);

        if (lx200cmd->startsWith(this->LX200Commands.slewRA,Qt::CaseSensitive)==1) {
            assembledString->clear();
            numSubStr->clear();
            if (sendSimpleCoordinates==false) {
                numSubStr->append(lx200cmd->right(8));
            } else {
                numSubStr->append(lx200cmd->right(7));
            }
            numSubStr->replace('.',":");
            numericalList=numSubStr->split(':',QString::SkipEmptyParts,Qt::CaseSensitive);
            rah=numericalList[0].toDouble();
            ram=numericalList[1].toDouble();
            if (sendSimpleCoordinates==false) {
                ras=numericalList[2].toDouble();
            } else {
                ras = 0.0;
            }
            this->receivedRAFromLX =(rah+ram/60.0+ras/3600.0)*15.0;
            numSubStr->clear();
            gotRACoordinates = true;
            assembledString->append("1");
            this->sendCommand(2);
            // got RA coordinates from LX200 ...
        }
        if (lx200cmd->startsWith(this->LX200Commands.slewDecl ,Qt::CaseSensitive)==1) {
            assembledString->clear();
            numSubStr->clear();
            if (sendSimpleCoordinates==false) {
                numSubStr->append(lx200cmd->right(9));
            } else {
                numSubStr->append(lx200cmd->right(6));
            }
            if (numSubStr->startsWith("-"))  {
                declSign = -1;
            } else {
                declSign = 1;
            }
            decldeg=(numSubStr->left(3)).toDouble();
            numSubStr->clear();
            numSubStr->append(lx200cmd->right(5));
            declmin=(numSubStr->left(2)).toDouble();
            if (sendSimpleCoordinates==false) {
                declsec=(numSubStr->right(2)).toDouble();
            } else {
                declsec = 0.0;
            }
            this->receivedDeclFromLX =declSign*(fabs(decldeg)+declmin/60.0+declsec/3600.0);
            numSubStr->clear();
            this->gotDeclCoordinates = true;
            assembledString->append("1");
            this->sendCommand(2);
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.slewPossible, Qt::CaseSensitive)==0) {         
            if ((this->gotDeclCoordinates==true) && (this->gotRACoordinates==true)) {
                assembledString->clear();
                waitTimer = new QElapsedTimer();
                waitTimer->start();
                do {
                    QCoreApplication::processEvents(QEventLoop::AllEvents,25);
                } while (waitTimer->elapsed() < 25);
                delete waitTimer; // just wait for 25 ms ...
                this->gotDeclCoordinates=false;
                this->gotRACoordinates=false;
                assembledString->append("0");
                this->sendCommand(2);
                emit RS232slew();
                QCoreApplication::processEvents(QEventLoop::AllEvents,25);

            }
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.syncCommand, Qt::CaseSensitive)==0) {
            if ((this->gotDeclCoordinates==true) && (this->gotRACoordinates==true)) {           
                assembledString->clear();
                waitTimer = new QElapsedTimer();
                waitTimer->start();
                do {
                    QCoreApplication::processEvents(QEventLoop::AllEvents,25);
                } while (waitTimer->elapsed() < 25);
                delete waitTimer; // just wait for 25 ms ...
                this->gotDeclCoordinates=false;
                this->gotRACoordinates=false;
                assembledString->append("---#");
                // now set the global coordinates in g_AllData to receivedRA and received Decl
                this->sendCommand(2);
                emit RS232sync();
                QCoreApplication::processEvents(QEventLoop::AllEvents,25);
            }
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.getDecl, Qt::CaseSensitive)==0) {
            // returns actual scope declination as "sDD*MM’SS#"
            this->assembleDeclinationString();
            assembledString->append(replyStrLX->toLatin1());
            this->msgDeclString->clear();
            this->msgDeclString->append(assembledString);
            this->sendCommand(1);
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.getRA, Qt::CaseSensitive)==0) {
            // returns actual scope RA as "HH:MM:SS#"
            this->assembleRAString();
            assembledString->append(replyStrLX->toLatin1());
            this->msgRAString->clear();
            this->msgRAString->append(assembledString);
            this->sendCommand(0);
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.stopMotion, Qt::CaseSensitive)==0) {
            emit this->RS232stopMotion();
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.moveEast, Qt::CaseSensitive)==0) {
            emit this->RS232moveEast();
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.moveWest, Qt::CaseSensitive)==0) {
            emit this->RS232moveWest();
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.moveNorth, Qt::CaseSensitive)==0) {
            emit this->RS232moveNorth();
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.moveSouth, Qt::CaseSensitive)==0) {
            emit this->RS232moveSouth();
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.stopMoveEast, Qt::CaseSensitive)==0) {
            emit this->RS232stopMoveEast();
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.stopMoveWest, Qt::CaseSensitive)==0) {
            emit this->RS232stopMoveWest();
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.stopMoveNorth, Qt::CaseSensitive)==0) {
            emit this->RS232stopMoveNorth();
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.stopMoveSouth, Qt::CaseSensitive)==0) {
            emit this->RS232stopMoveSouth();
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.setCenterSpeed, Qt::CaseSensitive)==0) {
            emit this->RS232centerSpeed();
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.setGuideSpeed, Qt::CaseSensitive)==0) {
            emit this->RS232guideSpeed();
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.setFindSpeed, Qt::CaseSensitive)==0) {
            emit this->RS232findSpeed();
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.setGOTOSpeed, Qt::CaseSensitive)==0) {
            emit this->RS232gotoSpeed();
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.getHiDef, Qt::CaseSensitive)==0) {
            // ignore this as we are always sending in high resolution
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.getCalendarFormat, Qt::CaseSensitive)==0) {
            assembledString->append(QString::number(24));
            assembledString->append("#");
            this->sendCommand(2);
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.getName, Qt::CaseSensitive)==0) {
            assembledString->append(g_AllData->getSiteName().toLatin1());
            assembledString->append("#");
            this->sendCommand(2);
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.getTrackingRate, Qt::CaseSensitive)==0) {
            assembledString->append("50.0#"); // completely pointless as we are using steppers, not synchro drives
            this->sendCommand(2);
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.getDate, Qt::CaseSensitive)==0) {
            // nothing yet
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.getLatitude, Qt::CaseSensitive)==0) {
            llat=g_AllData->getSiteCoords(0);
            if (llat < 0) {
                assembledString->append("-");
            } else {
                assembledString->append("+");
            }
            toDegs=(floor(fabs(llat)));
            if (toDegs < 10) {
                assembledString->append("0");
            }
            assembledString->append(QString::number(toDegs));
            assembledString->append("*");
            toMins=fabs(llat)-fabs(floor(llat));
            coordMins=round(toMins*60);
            if (coordMins < 10) {
                assembledString->append("0");
            }
            assembledString->append(QString::number(coordMins));
            assembledString->append("#");
            this->sendCommand(2);
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.getLongitude, Qt::CaseSensitive)==0) {
            llong=g_AllData->getSiteCoords(1);
            if (llong < 0) {
                assembledString->append("-");
            } else {
                assembledString->append("+");
            }
            toDegs=(floor(fabs(llong)));
            if (toDegs < 10) {
                assembledString->append("00");
            }
            if (toDegs < 100) {
                assembledString->append("0");
            }
            assembledString->append(QString::number(toDegs));
            assembledString->append("*");
            toMins=fabs(llong)-fabs(floor(llong));
            coordMins=round(toMins*60);
            if (coordMins < 10) {
                assembledString->append("0");
            }
            assembledString->append(QString::number(coordMins));
            assembledString->append("#");
            this->sendCommand(2);
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.getUTCOffset, Qt::CaseSensitive)==0) {
            lutc=g_AllData->getSiteCoords(2);
            if (lutc < 0) {
                assembledString->append("-");
            } else {
                assembledString->append("+");
            }
            if (fabs(lutc) < 10) {
                assembledString->append("0");
            }
            assembledString->append(QString::number(fabs(lutc)));
            assembledString->append("#");
            this->sendCommand(2);
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.getLocalTime, Qt::CaseSensitive)==0) {
            // nothing yet
        }
        if (lx200cmd->startsWith(this->LX200Commands.setLongitude, Qt::CaseSensitive) == 1) {
            assembledString->clear();
            numSubStr->clear();
            numSubStr->append(lx200cmd->right(lx200cmd->length()-2));
            numericalList=numSubStr->split('*',QString::SkipEmptyParts,Qt::CaseSensitive);
            coordDeg=numericalList[0].toDouble();
            coordMin=numericalList[1].toDouble();
            llong = coordDeg+coordMin/60.0;
            if (llong > 180) {
                llong=360.0-llong;
            }
            qDebug() << "Longitude is received as: " << llong;
            this->gotLong = true;
            if ((this->gotLong == true) && (this->gotLat == true) && (this->gotUTCOffset == true)) {
                this->setLocalization();
            }
            assembledString->append("1");
            this->sendCommand(2);
        }
        if (lx200cmd->startsWith(this->LX200Commands.setLatitude, Qt::CaseSensitive) == 1) {
            assembledString->clear();
            numSubStr->clear();
            numSubStr->append(lx200cmd->right(lx200cmd->length()-2));
            if (numSubStr->startsWith('-') == true) {
                coordSign = -1;
            } else {
                coordSign = 1;
            }
            numericalList=numSubStr->split('*');
            coordDeg=abs(numericalList[0].toDouble());
            coordMin=(numericalList[1].toDouble());
            llat=coordSign*(coordDeg+coordMin/60.0);
            qDebug() << "Latitude is received as: " << llat;
            this->gotLat = true;
            if ((this->gotLong == true) && (this->gotLat == true) && (this->gotUTCOffset == true)) {
                this->setLocalization();
            }
            assembledString->append("1");
            this->sendCommand(2);
            numSubStr->clear();

        }
        if (lx200cmd->startsWith(LX200Commands.setUTCOffset,Qt::CaseSensitive) == true) {
            assembledString->clear();
            numSubStr->clear();
            numSubStr->append(lx200cmd->right(lx200cmd->length()-2));
            lutc = numSubStr->toInt();
            qDebug() << "UTCOffset: " << lutc;
            this->gotUTCOffset = true;
            if ((this->gotLong == true) && (this->gotLat == true) && (this->gotUTCOffset == true)) {
                this->setLocalization();
            }
            assembledString->append("1");
            this->sendCommand(2);
            numSubStr->clear();
        }
        if (lx200cmd->startsWith(LX200Commands.setLocalTime,Qt::CaseSensitive) == true) { // receives local time and sets the pi & it's hardware clock to this time
            assembledString->clear();
            assembledString->append("1");
            this->sendCommand(2);
            numSubStr->clear();
            numSubStr->append(lx200cmd->right(lx200cmd->length()-2));
            numericalList.clear();
            numericalList=numSubStr->split(':',QString::SkipEmptyParts,Qt::CaseSensitive);
            lhh=abs(numericalList[0].toDouble());
            lmm=numericalList[1].toDouble();
            lss=numericalList[2].toDouble();
            this->timeString->clear();
            helper->setNum((int)(lhh));
            this->timeString->append(helper);
            helper->clear();
            this->timeString->append(":");
            helper->setNum((int)lmm);
            this->timeString->append(helper);
            helper->clear();
            this->timeString->append(":");
            helper->setNum((int)lss);
            this->timeString->append(helper);
            helper->clear();
            this->gotTime = true;
            if (this->gotDate == true) {
                if (g_AllData->getTimeFromLX200Flag() == true) {
                    setSystemDateAndTime();
                }
            }
            numSubStr->clear();
        }
        if (lx200cmd->startsWith(LX200Commands.setLocalDate) == true) { // receives a date and sets the clock to that date
            numSubStr->clear();
            numSubStr->append(lx200cmd->right(lx200cmd->length()-2));
            numericalList.clear();
            numericalList=numSubStr->split('/',QString::SkipEmptyParts,Qt::CaseSensitive);
            lmonth=abs(numericalList[0].toDouble());
            lday=numericalList[1].toDouble();
            lyear=numericalList[2].toDouble();
            this->dateString->clear();
            helper->clear();
            helper->setNum((int)(lyear+2000));
            this->dateString->append(helper);
            helper->clear();
            this->dateString->append("-");
            helper->setNum((int)lmonth);
            this->dateString->append(helper);
            helper->clear();
            this->dateString->append("-");
            helper->setNum((int)lday);
            this->dateString->append(helper);
            helper->clear();
            this->gotDate = true;
            if (this->gotTime == true) {
                if (g_AllData->getTimeFromLX200Flag() == true) {
                    setSystemDateAndTime();
                }
            }
            waitTimer = new QElapsedTimer();
            assembledString->clear();
            assembledString->append("1");
            this->sendCommand(2);
            waitTimer->start();
            do {
                QCoreApplication::processEvents(QEventLoop::AllEvents,100);
            } while (waitTimer->elapsed() < 100);
            assembledString->clear();
            assembledString->append("Updating Planetary Data       #");
            this->sendCommand(2);
            waitTimer->restart();
            do {
                QCoreApplication::processEvents(QEventLoop::AllEvents,100);
            } while (waitTimer->elapsed() < 100);
            assembledString->clear();
            assembledString->append("                              #");
            this->sendCommand(2);
            delete waitTimer; // just wait for 250 ms ...
            numSubStr->clear();
        }
    }
    delete helper;
    delete lx200cmd;
    delete numSubStr;
    return true;
}

//-----------------------------------------------

void lx200_communication::sendCommand(short what) {
    if (what == 0) {
        emit this->logRASent();
        emit this->clientRASent(msgRAString);
     } else if (what == 1) {
        emit this->logDeclSent();
        emit this->clientDeclSent(msgDeclString);
     } else {
        if (assembledString->length() != 0) {
            emit this->logCommandSent();
            emit this->clientCommandSent(assembledString);
            qDebug() << "reply: " << assembledString->toLatin1();
        }
    }
    QCoreApplication::processEvents(QEventLoop::AllEvents,500);
}

//-----------------------------------------------
void lx200_communication::assembleDeclinationString(void) {
    QString *helper;
    double currDecl, remainder;
    int declDeg, declMin, declSec,sign=1;

    replyStrLX->clear();
    currDecl = g_AllData->getActualScopePosition(1);
    if (currDecl < 0) {
        sign = -1;
    } else {
        sign = 1;
    }
    declDeg=(int)(sign*floor(fabs(currDecl)));
    remainder = fabs(currDecl-((double)declDeg));
    declMin=(int)(floor(remainder*60.0));
    remainder = remainder*60.0-declMin;
    declSec=round(remainder*60);
    helper = new QString();
    if (sign == 1) {
        replyStrLX->insert(0,'+');
    } else {
        replyStrLX->insert(0,'-');
    }

    if ((abs(declDeg)) < 10) {
        replyStrLX->insert(1,'0');
    }
    helper->setNum(abs(declDeg));
    replyStrLX->append(helper);
    helper->clear();
    replyStrLX->append("*");
    if (declMin < 10) {
        replyStrLX->append("0");
    }
    helper->setNum(declMin);
    replyStrLX->append(helper);
    helper->clear();
    if (sendSimpleCoordinates==false) {
        replyStrLX->append(":");
        if (declSec < 10) {
            replyStrLX->append("0");
        }
        helper->setNum(declSec);
        replyStrLX->append(helper);
    }
    replyStrLX->append("#");
    delete helper;
}

//---------------------------------------------------

void lx200_communication::assembleRAString(void) {
    QString *helper;
    double currRA, remainder, RAInHours;
    int RAHrs, RAMin, RASec;

    replyStrLX->clear();
    currRA = g_AllData->getActualScopePosition(2);
    RAInHours = currRA/360.0*24.0;
    if (RAInHours > 24) {
        RAInHours -= 24;
    }
    RAHrs = floor(RAInHours);
    RAMin = floor((RAInHours - RAHrs)*60.0);
    remainder = ((RAInHours - RAHrs)*60.0) - RAMin;
    RASec = round(remainder*60.0);
    helper = new QString();
    if (RAHrs < 10) {
        replyStrLX->append("0");
    }
    helper->setNum(RAHrs);
    replyStrLX->append(helper);
    helper->clear();
    replyStrLX->append(":");
    if (RAMin < 10) {
        replyStrLX->append("0");
    }
    helper->setNum(RAMin);
    replyStrLX->append(helper);
    helper->clear();
    if (sendSimpleCoordinates==false) {
        replyStrLX->append(":");
        if (RASec < 10) {
            replyStrLX->append("0");
        }
        helper->setNum(RASec);
        replyStrLX->append(helper);
        helper->clear();
    }
    replyStrLX->append("#");
    delete helper;
}

//---------------------------------------------------

double lx200_communication::getReceivedCoordinates(short what) {
    double retval;

    if (what == 0) {
        retval = this->receivedRAFromLX;
    } else {
        retval = this->receivedDeclFromLX;
    }
    return retval;
}

//---------------------------------------------------

QString* lx200_communication::getLX200Command(void) {
    return this->lastSubCmd;
}

//---------------------------------------------------

QString* lx200_communication::getLX200Response(void) {
    return this->assembledString;
}

//---------------------------------------------------

QString* lx200_communication::getLX200ResponseRA(void) {
    return this->msgRAString;
}
//---------------------------------------------------

QString* lx200_communication::getLX200ResponseDecl(void) {
    return this->msgDeclString;
}
//---------------------------------------------------

void lx200_communication::setNumberFormat(bool isSimple) {
    this->sendSimpleCoordinates=isSimple;
}

//----------------------------------------------------

void lx200_communication::setSystemDateAndTime(void) {
    QString *systemCmd, *helper;
    time_t rawtime;
    struct tm *ptm;
    int year, month, day, hr, min, sec;

    systemCmd = new QString();
    helper = new QString();
    systemCmd->append("sudo timedatectl set-time ");
    systemCmd->append(this->dateString->toLatin1());
    system(systemCmd->toLatin1());
    usleep(250);
    systemCmd->clear();
    systemCmd->append("sudo timedatectl set-time ");
    systemCmd->append(this->timeString->toLatin1());
    system(systemCmd->toLatin1());

    // clock was set to local time ... now comes the trick -
    // convert local time and date to GM time, and set the HW clock to UTC
    time(&rawtime);
    rawtime = rawtime+this->lutc*3600+2.2; // 200 ms were spent waiting for the replies from LX200, two seconds are lost in communication
    ptm = gmtime(&rawtime);
    year = ptm->tm_year+1900;
    month = ptm->tm_mon + 1;
    day = ptm->tm_mday;
    hr = ptm->tm_hour;
    min = ptm->tm_min;
    sec = ptm->tm_sec;
    this->dateString->clear();
    helper->clear();
    helper->setNum((int)(year));
    this->dateString->append(helper);
    helper->clear();
    this->dateString->append("-");
    helper->setNum((int)month);
    this->dateString->append(helper);
    helper->clear();
    this->dateString->append("-");
    helper->setNum((int)day);
    this->dateString->append(helper);
    helper->clear();
    this->timeString->clear();
    helper->setNum((int)(hr));
    this->timeString->append(helper);
    helper->clear();
    this->timeString->append(":");
    helper->setNum((int)min);
    this->timeString->append(helper);
    helper->clear();
    this->timeString->append(":");
    helper->setNum((int)sec);
    this->timeString->append(helper);
    helper->clear();
    systemCmd->clear();
    systemCmd->append("sudo timedatectl set-time ");
    systemCmd->append(this->dateString->toLatin1());
    system(systemCmd->toLatin1());
    qDebug() << "Sent UTC-date: " << systemCmd->toLatin1();
    usleep(250);
    systemCmd->clear();
    systemCmd->append("sudo timedatectl set-time ");
    systemCmd->append(this->timeString->toLatin1());
    system(systemCmd->toLatin1());
    qDebug() << "Sent UTC-time: " << systemCmd->toLatin1();
    usleep(250);
    system("sudo hwclock -w");
    usleep(250);
    delete helper;
    delete systemCmd;
    qDebug() << "Hardwareclock set ...";
}

//-------------------------------------------------------------
// this one sets the received longitude, latitude and timezone
void lx200_communication::setLocalization(void) {
    if (g_AllData->getTimeFromLX200Flag() == true) {
        g_AllData->setSiteParams(this->llat, this->llong, this->lutc);
        g_AllData->storeGlobalData();
        qDebug() << "Stored location...";
        emit this->localizationSet();
    }
}



