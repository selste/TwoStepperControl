#include "lx200_communication.h"
#include "tsc_globaldata.h"
#include <QDebug>

extern TSC_GlobalData *g_AllData;

//--------------------------------------------------------

lx200_communication::lx200_communication(void) {
    this->portIsUp=false;
    rs232port.setPortName("/dev/ttyS0");
    rs232port.setBaudRate(QSerialPort::Baud9600);
    rs232port.setDataBits(QSerialPort::Data8);
    rs232port.setParity(QSerialPort::NoParity);
    rs232port.setStopBits(QSerialPort::OneStop);
    rs232port.setFlowControl(QSerialPort::SoftwareControl);
    replyStrLX = new QString();
    this->serialData = new QByteArray();
    LX200Commands.getDecl = QString("#:GD#");
    LX200Commands.getRA = QString("#:GR#");
    LX200Commands.getHiDef = QString("#:U#");
    LX200Commands.getRAandDecl = QString("#:GR#:GD#");
}

//--------------------------------------------------------

lx200_communication::~lx200_communication(void) {
    portIsUp = 0;
    rs232port.clear(QSerialPort::AllDirections);
    rs232port.close();
    delete replyStrLX;
    delete serialData;
}

//--------------------------------------------------------

void lx200_communication::shutDownPort(void) {
    qDebug() << "Break enabled:" << rs232port.setBreakEnabled(true);
    portIsUp = 0;
    rs232port.clear(QSerialPort::AllDirections);
    rs232port.close();
    replyStrLX->clear();
}

//--------------------------------------------------------

void lx200_communication::openPort(void) {
    portIsUp = 1;
    qDebug() << "Trying to open serial port";
    if (!rs232port.open(QIODevice::ReadWrite)) {
        qDebug() << "Open port failed";
        portIsUp = 0;
    } else {
        qDebug() << "Open port succeeded";
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
    qint64 charsToBeRead, charsRead=0;
    QString *incomingCommand;

    charsToBeRead=rs232port.bytesAvailable();
    incomingCommand = new QString();
    if (charsToBeRead > 0) {
        this->serialData->append(rs232port.readAll());
        charsRead=serialData->length();
        if (charsRead != -1) {
            incomingCommand->append(serialData->data());
            this->serialData->clear();
            this->handleBasicLX200Protocol(*incomingCommand);
        }
    }
    delete incomingCommand;
    return charsRead;
}

//--------------------------------------------------------

bool lx200_communication::handleBasicLX200Protocol(QString cmd) {
    QString *assembledString;
    qint64 bytesWritten, replyLength;
    bool commandRecognized = 0;

    qDebug() << "LX200 request:" << (cmd.toLatin1());
    if ((cmd.length() == 1) && ((int)(cmd.toLatin1()[0])==6)){
    // if LX200 sends <ACK> -> reply with P for forks, G for german equatorials or A for Alt/Az
        bytesWritten = rs232port.write("P");
        qDebug() << "Sent 'P' as a reply for <ACK> ...";
    }

    if (QString::compare(cmd.toLatin1(),this->LX200Commands.getDecl, Qt::CaseSensitive)==0) {
        // returns actual scope declination as "sDD*MM’SS#"
        commandRecognized = 1;
        this->assembleDeclinationString();
        assembledString = new QString(replyStrLX->toLatin1());
    }
    if (QString::compare(cmd.toLatin1(),this->LX200Commands.getRA, Qt::CaseSensitive)==0) {
        // returns actual scope RA as "HH:MM:SS#"
        commandRecognized = 1;
        this->assembleRAString();
        assembledString = new QString(replyStrLX->toLatin1());
    }
     if (QString::compare(cmd.toLatin1(),this->LX200Commands.getRAandDecl, Qt::CaseSensitive)==0) {
         commandRecognized = 1;
         this->assembleRAString();
         assembledString = new QString(replyStrLX->toLatin1());
         assembledString->append("\n");
         this->assembleDeclinationString();
         assembledString->append(replyStrLX->toLatin1());
     }
    if (QString::compare(cmd.toLatin1(),this->LX200Commands.getHiDef, Qt::CaseSensitive)==0) {
        commandRecognized = 0;
        // ignore this as we are always sending in high resolution
    }
    if (commandRecognized == true) {
        replyLength=assembledString->length();
        qDebug() << "Sending: " << assembledString->toLatin1();
        bytesWritten = rs232port.write((assembledString->toLatin1()));
        delete assembledString;
        if (replyLength == bytesWritten) {
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}

//-----------------------------------------------

void lx200_communication::assembleDeclinationString(void) {
    QString *helper;
    double currDecl, remainder;
    int declDeg, declMin, declSec,sign;

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
    if (abs(declDeg) < 10) {
        if (declDeg >= 0) {
            replyStrLX->append("+0");
        } else {
            replyStrLX->append("-0");
        }
        helper->setNum(abs(declDeg));
        replyStrLX->append(helper);
        helper->clear();
    } else {
        replyStrLX->setNum(declDeg);
    }
    replyStrLX->append("*");
    if (declMin < 10) {
        replyStrLX->append("0");
    }
    helper->setNum(declMin);
    replyStrLX->append(helper);
    helper->clear();
    replyStrLX->append(":");
    if (declSec < 10) {
        replyStrLX->append("0");
    }
    helper->setNum(declSec);
    replyStrLX->append(helper);
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
    replyStrLX->append(":");
    if (RASec < 10) {
        replyStrLX->append("0");
    }
    helper->setNum(RASec);
    replyStrLX->append(helper);
    helper->clear();
    replyStrLX->append("#");
}

//---------------------------------------------------
