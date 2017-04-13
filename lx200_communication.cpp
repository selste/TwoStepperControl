#include "lx200_communication.h"
#include "tsc_globaldata.h"
#include <QDebug>
#include <unistd.h>


extern TSC_GlobalData *g_AllData;

//--------------------------------------------------------

lx200_communication::lx200_communication(void) {
    this->portIsUp=false;
    rs232port.setPortName("/dev/ttyS0");
    rs232port.setBaudRate(QSerialPort::Baud9600);
    rs232port.setDataBits(QSerialPort::Data8);
    rs232port.setParity(QSerialPort::NoParity);
    rs232port.setStopBits(QSerialPort::OneStop);
    rs232port.setFlowControl(QSerialPort::NoFlowControl);
    replyStrLX = new QString();
    assembledString = new QString();
    this->serialData = new QByteArray();
    this->incomingCommand = new QString();
    this->msgRAString = new QString();
    this->msgDeclString = new QString();
    this->subCmd = new QString();
    this->lastSubCmd = new QString(); // a buffer holding the last command, just for protocols
    receivedRAFromLX = 0.0;
    receivedDeclFromLX = 0.0;
    gotRACoordinates = false;
    gotDeclCoordinates = false;
    sendSimpleCoordinates=false; // determine the reponse format as ddd:mm or ddd:mm:ss
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
}

//--------------------------------------------------------

lx200_communication::~lx200_communication(void) {
    if (portIsUp == 1) {
        rs232port.setBreakEnabled(true);
        rs232port.clear(QSerialPort::AllDirections);
        rs232port.close();
        portIsUp = 0;
    }
    delete replyStrLX;
    delete serialData;
    delete incomingCommand;
    delete assembledString;
    delete msgRAString;
    delete msgDeclString;
    delete subCmd;
    delete lastSubCmd;
}

//--------------------------------------------------------

void lx200_communication::shutDownPort(void) {
    rs232port.setBreakEnabled(true);
    portIsUp = 0;
    rs232port.clear(QSerialPort::AllDirections);
    rs232port.close();
    replyStrLX->clear();
}

//--------------------------------------------------------

void lx200_communication::openPort(void) {
    portIsUp = 1;
    if (!rs232port.open(QIODevice::ReadWrite)) {
        portIsUp = 0;
    } else {
        rs232port.setBreakEnabled(false);
        portIsUp = 1;
        rs232port.clear(QSerialPort::AllDirections);
    }
    replyStrLX->clear();
}

//--------------------------------------------------------

bool lx200_communication::getPortState(void) {
    return portIsUp;
}

//--------------------------------------------------------

qint64 lx200_communication::getDataFromSerialPort(void) {
    qint64 charsToBeRead=0;
    qint64 charsRead=0;
    QStringList *subCmdList;
    int cmdCounter;
    QElapsedTimer *waitTimer;

    waitTimer = new QElapsedTimer();
    waitTimer->start();
    do {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 25);
    } while (waitTimer->elapsed() < 25);
    delete waitTimer; // just wait for 25 ms if data come in ...
    charsToBeRead=rs232port.bytesAvailable();
    if (charsToBeRead > 1) {
        this->serialData->append(rs232port.readAll());
        if (charsRead != -1) {
            this->incomingCommand->append(serialData->data());
            this->lastSubCmd->clear();
            this->lastSubCmd->append(this->incomingCommand->toLatin1());
            this->serialData->clear();
            subCmdList = new QStringList(this->incomingCommand->split("#:", QString::SkipEmptyParts));
            if (subCmdList->isEmpty()==false) {
                for (cmdCounter = 0; cmdCounter < (subCmdList->length()-1); cmdCounter++) {
                    this->subCmd->append(subCmdList->at(cmdCounter));
                    this->subCmd->remove("#");
                    this->handleBasicLX200Protocol(*subCmd);                
                    emit this->RS232CommandReceived();
                    this->subCmd->clear();
                }      
                this->subCmd->append(subCmdList->at(subCmdList->length()-1));
                delete subCmdList;
                this->incomingCommand->clear();
                if (this->subCmd->right(1)!="#") {
                    this->incomingCommand->append("#:");
                    this->incomingCommand->append(this->subCmd); // remeber the command until next time
                } else {
                    this->subCmd->remove("#:");
                    this->subCmd->remove("#");
                    this->handleBasicLX200Protocol(*subCmd);
                    emit this->RS232CommandReceived();
                    this->subCmd->clear();
                }
            }
        }
    }
    return charsRead;
}

//--------------------------------------------------------

bool lx200_communication::handleBasicLX200Protocol(QString cmd) {
    QString *lx200cmd, *numSubStr;
    QStringList commandList,numericalList;
    int numberOfCommands, cmdCounter;
    double rah,ram,ras,decldeg,declmin,declsec;
    short declSign;
    QElapsedTimer *waitTimer;

    lx200cmd = new QString();
    commandList = cmd.split('#',QString::SkipEmptyParts,Qt::CaseSensitive);
    numberOfCommands=commandList.count();

    for (cmdCounter = 0; cmdCounter < numberOfCommands; cmdCounter++) {
        lx200cmd->clear();
        assembledString->clear();
        lx200cmd->append(commandList[cmdCounter]);

        if (lx200cmd->startsWith(this->LX200Commands.slewRA,Qt::CaseSensitive)==1) {
            if (sendSimpleCoordinates==false) {
                numSubStr = new QString(lx200cmd->right(8));
            } else {
                numSubStr = new QString(lx200cmd->right(7));
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
            delete numSubStr;        
            gotRACoordinates = true;
            assembledString->append(QString::number(1));
            this->sendCommand(2);
            // got RA coordinates from LX200 ...
        }
        if (lx200cmd->startsWith(this->LX200Commands.slewDecl ,Qt::CaseSensitive)==1) {
            if (sendSimpleCoordinates==false) {
                numSubStr = new QString(lx200cmd->right(9));
            } else {
                numSubStr = new QString(lx200cmd->right(6));                
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
            delete numSubStr;
            this->gotDeclCoordinates = true;
            assembledString->append(QString::number(1));
            this->sendCommand(2);
            // got Decl coordinates from LX200 ...
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.slewPossible, Qt::CaseSensitive)==0) {         
            if ((this->gotDeclCoordinates==true) && (this->gotRACoordinates==true)) {
                waitTimer = new QElapsedTimer();
                waitTimer->start();
                do {
                    QCoreApplication::processEvents(QEventLoop::AllEvents, 25);
                } while (waitTimer->elapsed() < 25);
                delete waitTimer; // just wait for 25 ms ...
                this->gotDeclCoordinates=false;
                this->gotRACoordinates=false;
                assembledString->append(QString::number(0));
                emit RS232slew();
                this->sendCommand(2);
            }
        }
        if (QString::compare(lx200cmd->toLatin1(),this->LX200Commands.syncCommand, Qt::CaseSensitive)==0) {
            if ((this->gotDeclCoordinates==true) && (this->gotRACoordinates==true)) {           
                waitTimer = new QElapsedTimer();
                waitTimer->start();
                do {
                    QCoreApplication::processEvents(QEventLoop::AllEvents, 25);
                } while (waitTimer->elapsed() < 25);
                delete waitTimer; // just wait for 25 ms ...
                this->gotDeclCoordinates=false;
                this->gotRACoordinates=false;
                assembledString->append("M31 EX GAL MAG 35 SZ178.0'#");
                // now set the global coordinates in g_AllData to receivedRA and received Decl
                emit RS232sync();
                this->sendCommand(2);
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
    }
    delete lx200cmd;
    return true; // do something here later ...
}

//-----------------------------------------------

void lx200_communication::sendCommand(short what) {
    if (what == 0) {
        rs232port.write((msgRAString->toLatin1()));
        rs232port.flush();
        emit this->RS232RASent();
    } else if (what == 1) {
        rs232port.write((msgDeclString->toLatin1()));
        rs232port.flush();
        emit this->RS232DeclSent();
    } else {
        rs232port.write((assembledString->toLatin1()));
        rs232port.flush();
        emit this->RS232CommandSent();
    }
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
    declSec=round(remainder);
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
