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
#include "QtKineticStepper.h"
#include <unistd.h>
#include "tsc_globaldata.h"
#include <stdlib.h>
#include <math.h>
#include <QDebug>
#include "usb_communications.h"

extern TSC_GlobalData *g_AllData;
extern usbCommunications *amisInterface;

//-----------------------------------------------------------------------------

QtKineticStepper::QtKineticStepper(void){

    this->sendCommandToAMIS("e",1); // enable stepper
    this->hBoxSlewEnded=false;
    this->stopped=true;
    this->gearRatio = 1;
    this->microsteps = 2;
    qDebug() << "Called Declination constructor";
}

//-----------------------------------------------------------------------------

QtKineticStepper::~QtKineticStepper(void){
    this->stopped=true;
    this->sendCommandToAMIS("x"); // stop steppers
    this->sendCommandToAMIS("e",0); // disable steppers
}

//-----------------------------------------------------------------------------
void QtKineticStepper::setGearRatioAndMicrosteps(double gr, double ms) {
    long lms;

    this->gearRatio=gr;
    lms = (long)ms;
    if ((lms != 4) && (lms != 8) && (lms != 16) && (lms != 32) && (lms != 64) && (lms != 128) && (lms != 256)) {
        lms = 16;
    }
    this->sendCommandToAMIS("x"); // stop steppers
    this->sendCommandToAMIS("m",lms);
    usleep(50);
    this->microsteps=lms;
}

//-----------------------------------------------------------------------------
// this routine changes the microstepping ration; for the phidget drivers, it has no effect
void QtKineticStepper::changeMicroSteps(double ms) {
    long lms;

    lms = (long)ms;
    if ((lms != 4) && (lms != 8) && (lms != 16) && (lms != 32) && (lms != 64) && (lms != 128) && (lms != 256)) {
        lms = 16;
    }
    this->sendCommandToAMIS("x"); // stop steppers
    this->sendCommandToAMIS("m",lms);
    usleep(50);
    this->microsteps=lms;
}

//-----------------------------------------------------------------------------

void QtKineticStepper::setInitialParamsAndComputeBaseSpeed(double lacc, double lcurr) {
    double smax, amax, smin;

    smax = 25000;
    amax = 20000;
    smin = 0;
    if (lacc > amax) {
        this->acc = amax;
    } else {
        this->acc = lacc;
    }
    if (lcurr <= 3) {
        this->currMax=lcurr;
    } else {
        this->currMax = 3;
    }
    this->sendCommandToAMIS("a", this->acc);
    this->sendCommandToAMIS("c",(long)(this->currMax*1000));
    usleep(100);
    this->stepsPerSecond=round(g_AllData->getCelestialSpeed()*(this->gearRatio)*(this->microsteps));
    if (this->stepsPerSecond < 1) {
        this->speedMax = 1; // a speed of less than 1 microsteps is not achievable - it is physics, baby ...
        this->stepsPerSecond = this->speedMax;
    }
    this->speedMax=this->stepsPerSecond;
    this->sendCommandToAMIS("v",(long)(this->speedMax));
}

//-----------------------------------------------------------------------------
void QtKineticStepper::travelForNSteps(long steps,short direction, int factor,bool isHBSlew) {
    const short directionfactor = -1; // change to switch directions of the drive

    this->hBoxSlewEnded = false;
    this->isHBoxSlew=isHBSlew;
    this->speedMax=round(factor*g_AllData->getCelestialSpeed()*(this->gearRatio)*(this->microsteps));
    if (direction < 0) {
        direction = -1;
    } else {
        direction = 1;
    }
    this->sendCommandToAMIS("v",(long)(this->speedMax));
    this->sendCommandToAMIS("z");
    this->sendCommandToAMIS("s", (long)g_AllData->getMFlipDecSign()*directionfactor*direction*steps);
    this->sendCommandToAMIS("o");
    this->stopped = false;
}

//-----------------------------------------------------------------------------
// same as above for guiding with ST4 and so on
void QtKineticStepper::travelForNSteps(short direction, float factor) {
    const short directionfactor = -1; // change to switch directions of the drive

    this->speedMax=round(factor*g_AllData->getCelestialSpeed()*(this->gearRatio)*(this->microsteps));
    if (direction < 0) {
        direction = -1;
    } else {
        direction = 1;
    }
    this->sendCommandToAMIS("v",(long)(this->speedMax));
    this->sendCommandToAMIS("z");
    this->sendCommandToAMIS("s", (long)g_AllData->getMFlipDecSign()*directionfactor*direction*1000000);
    this->sendCommandToAMIS("o");
    this->stopped = false;
}

//----------------------------------------------------------------------------------
void QtKineticStepper::resetSteppersAfterStop(void) { // this function is called once it was detected that the steppers stopped moving
    this->stopped = true;
    this->speedMax=g_AllData->getCelestialSpeed()*(this->gearRatio)*(this->microsteps);
    this->sendCommandToAMIS("v",(long)(this->speedMax));
    if (this->isHBoxSlew == true) {
        this->hBoxSlewEnded=true;
        this->isHBoxSlew = false;
    }
}

//-----------------------------------------------------------------------------
double QtKineticStepper::getKineticsFromController(short whichOne) {
    double retval;

    switch (whichOne) {
    case 1:
        retval = (double)(this->sendCommandToAMIS("f9").toLong()/1000.0);
        break;
    case 2:
        retval = (double)(this->sendCommandToAMIS("f8").toLong());
        break;
    case 3:
        retval = (double)(this->sendCommandToAMIS("f7").toLong());
        break;
    case 4:
        retval = (this->speedMin);
        break;
    default:
        retval=-1;
    }
    return retval;
}

//-----------------------------------------------------------------------------

void QtKineticStepper::setStepperParams(double val, short whichOne) {

    switch (whichOne) {
    case 1:
        this->acc=val;
        this->sendCommandToAMIS("a", this->acc);
        break;
    case 2:
        this->speedMax=val;
        this->sendCommandToAMIS("v", this->speedMax);
        break;
    case 3:
        if (val > 3) {
            val = 3;
        }
        this->currMax=val;
        this->sendCommandToAMIS("c", this->currMax*1000);
        usleep(100);
    }
    return;
}


//-----------------------------------------------------------------------------

void QtKineticStepper::shutDownDrive(void) {
    this->stopped=true;
    this->sendCommandToAMIS("x");
    this->sendCommandToAMIS("e",0);
}

//-----------------------------------------------------------------------------

bool QtKineticStepper::getStopped(void) {
    return (this->stopped);
}

//------------------------------------------------------------------------------
void QtKineticStepper::stopDrive(void) {
    this->sendCommandToAMIS("s",0);
    this->sendCommandToAMIS("x");
    this->sendCommandToAMIS("z");
    this->stopped=true;
}

//-------------------------------------------------------------------------------
void QtKineticStepper::changeSpeedForGearChange(void) {

    this->stepsPerSecond=round(g_AllData->getCelestialSpeed()*(this->gearRatio)*(this->microsteps));
    if (this->stepsPerSecond < 0) {
        this->speedMax = 0; // a speed of less than 0 microsteps is not achievable - it is physics, baby ...
        this->stepsPerSecond = this->speedMax;
    }
    if (this->stepsPerSecond > 50000) {
        this->speedMax = 50000; // a speed of more than 50000 microsteps is not achievable - it is physics, baby ...
        this->stepsPerSecond = this->speedMax;
    }
    this->speedMax=stepsPerSecond;
    this->sendCommandToAMIS("v", this->speedMax);
    // 360°/sidereal day in seconds*gear ratios*microsteps/steps
}

//-------------------------------------------------------------------------------

bool QtKineticStepper::hasHBoxSlewEnded(void) {
    return this->hBoxSlewEnded;
}

//--------------------------------------------------------------------------------
// two private routines to simplify communications with the AMIS
QString QtKineticStepper::sendCommandToAMIS(QString cmd, long val) {
    QString theCommand;
    QString theReply;

    theCommand.append(cmd);
    theCommand.append(QString::number(val, 10));
    amisInterface->sendCommand(theCommand,false);
    theReply.append(amisInterface->getReply(false));
   /* if (cmd != "f7") {
        qDebug() << "--- Command to Decl ---";
        qDebug() << "Sent: " << theCommand.toLatin1();
        qDebug() << "Received: " << theReply.toLatin1();
    }*/
    return theReply;
}

//--------------------------------------------------------------------------------
QString QtKineticStepper::sendCommandToAMIS(QString cmd) {
    QString theCommand;
    QString theReply;

    theCommand.append(cmd);
    amisInterface->sendCommand(theCommand,false);
    theReply.append(amisInterface->getReply(false));
 /*   if (cmd != "f7") {
        qDebug() << "--- Command to Decl ---";
        qDebug() << "Sent: " << theCommand.toLatin1();
        qDebug() << "Received: " << theReply.toLatin1();
    }*/
    return theReply;
}
