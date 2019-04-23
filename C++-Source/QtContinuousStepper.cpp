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
#include "QtContinuousStepper.h"
#include <stdlib.h>
#include <unistd.h>
#include "tsc_globaldata.h"
#include "usb_communications.h"
#include <math.h>
#include <QDebug>

extern TSC_GlobalData *g_AllData;
extern usbCommunications *amisInterface;

QtContinuousStepper::QtContinuousStepper(void){
    int sernum, version;

    if (g_AllData->getStepperDriverType() == 0 ) {
        this->SH = NULL;
        this->errorCreate = CPhidgetStepper_create(&SH);
        this->errorOpen = CPhidget_open((CPhidgetHandle)SH, -1);
        sleep(1);
        CPhidget_getSerialNumber((CPhidgetHandle)SH, &sernum);
        CPhidget_getDeviceVersion((CPhidgetHandle)SH, &version);
        sleep(1);
        CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 1); // enable steppers
        this->snumifk=sernum;
        this->vifk=version;
    }
    if (g_AllData->getStepperDriverType() == 1) {
        this->sendCommandToAMIS("e",1); // enable steppers
    }
    this->hBoxSlewEnded=false;
    this->isHBoxSlew = false;
    this->stopped=true;
    this->gearRatio = 1;
    this->microsteps = 2;
    qDebug() << "Called Right Ascension constructor";
}

//-----------------------------------------------------------------------------

QtContinuousStepper::~QtContinuousStepper(void){
    this->stopped=true;
    if (g_AllData->getStepperDriverType() == 0) {
        CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 0);
        sleep(1);
        CPhidget_close((CPhidgetHandle)SH);
        CPhidget_delete((CPhidgetHandle)SH);
    }
    if (g_AllData->getStepperDriverType() == 1) {
        this->sendCommandToAMIS("x"); // stop steppers
        this->sendCommandToAMIS("e",0); // disable steppers
    }
}

//-----------------------------------------------------------------------------
void QtContinuousStepper::setGearRatioAndMicrosteps(double gr, double ms) {
    long lms;

    this->gearRatio=gr;
    lms = (long)ms;
    if ((lms != 4) && (lms != 8) && (lms != 16) && (lms != 32) && (lms != 64) && (lms != 128) && (lms != 256)) {
        lms = 16;
    }
    if (g_AllData->getStepperDriverType() == 1) {
        this->sendCommandToAMIS("x"); // stop steppers
        this->sendCommandToAMIS("m",lms);
        usleep(50);
    } else {
        lms = 16;
    }
    this->microsteps=lms;
}

//-----------------------------------------------------------------------------
// this routine changes the microstepping ration; for the phidget drivers, it has no effect
void QtContinuousStepper::changeMicroSteps(double ms) {
    long lms;

    if (g_AllData->getStepperDriverType() == 1) {
        lms = (long)ms;
        if ((lms != 4) && (lms != 8) && (lms != 16) && (lms != 32) && (lms != 64) && (lms != 128) && (lms != 256)) {
            lms = 16;
        }
        this->sendCommandToAMIS("x"); // stop steppers
        this->sendCommandToAMIS("m",lms);
        usleep(50);
        this->microsteps=lms;
    }
}

//-----------------------------------------------------------------------------

void QtContinuousStepper::setInitialParamsAndComputeBaseSpeed(double lacc, double lcurr) {
    double smax, amax, smin;

    if (g_AllData->getStepperDriverType() == 0) {
        if ((this->errorCreate == 0) && (this->errorOpen == 0)) {
            CPhidgetStepper_getVelocityMax((CPhidgetStepperHandle)SH,0,&smax);
            CPhidgetStepper_getVelocityMin((CPhidgetStepperHandle)SH,0,&smin);
            CPhidgetStepper_getAccelerationMax((CPhidgetStepperHandle)SH,0,&amax);
            this->speedMin=smin;
            if (lacc > amax) {
                this->acc = amax;
            } else {
                this->acc = lacc;
            }
            if (lcurr <= 4) {
                this->currMax=lcurr;
            } else {
                this->currMax = 4;
            }
            CPhidgetStepper_setAcceleration((CPhidgetStepperHandle)SH,0,this->acc);
            CPhidgetStepper_setCurrentLimit((CPhidgetStepperHandle)SH,0,currMax);
            this->stepsPerSecond=round(g_AllData->getCelestialSpeed()*(this->gearRatio)*(this->microsteps));
            if (this->stepsPerSecond < 1) {
                this->speedMax = 1; // a speed of less than 1 microsteps is not achievable - it is physics, baby ...
                this->stepsPerSecond = this->speedMax;
            }
            this->speedMax=this->stepsPerSecond;
            CPhidgetStepper_setVelocityLimit((CPhidgetStepperHandle)SH,0,this->speedMax);
        }
    }
    if (g_AllData->getStepperDriverType() == 1) {
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
}

//----------------------------------------------
void QtContinuousStepper::startTracking(void) {

    this->speedMax=g_AllData->getCelestialSpeed()*(this->gearRatio*this->microsteps);
    if (g_AllData->getStepperDriverType() == 0) {
        CPhidgetStepper_setVelocityLimit((CPhidgetStepperHandle)SH,0,this->speedMax);
        CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 1);
        CPhidgetStepper_setCurrentPosition((CPhidgetStepperHandle)SH, 0, 0);
        CPhidgetStepper_setTargetPosition((CPhidgetStepperHandle)SH, 0, this->RADirection*(60*60*24*this->stepsPerSecond));
        // one day is maximum tracking time
        this->stopped = false;
    }
    if (g_AllData->getStepperDriverType() == 1) {
        this->sendCommandToAMIS("v",this->speedMax);
        this->sendCommandToAMIS("z");
        this->sendCommandToAMIS("s", this->RADirection*(60*60*24*this->stepsPerSecond));
        this->sendCommandToAMIS("o");
        this->stopped = false;
    }
}

//-----------------------------------------------
void QtContinuousStepper::travelForNSteps(long steps,short direction, int factor, bool isHBSlew) {
    const short directionfactor = -1; // switch *(-1) to change drive directions

    this->hBoxSlewEnded = false;
    this->isHBoxSlew = isHBSlew;
    this->speedMax=round(factor*g_AllData->getCelestialSpeed()*(this->gearRatio)*(this->microsteps));
    qDebug() << this->speedMax;
    qDebug() << this->microsteps;

    if (direction < 0) {
        direction = -1;
    } else {
        direction = 1;
    }
    if (g_AllData->getStepperDriverType() == 0) {
        CPhidgetStepper_setVelocityLimit((CPhidgetStepperHandle)SH,0,this->speedMax);
        CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 1);
        CPhidgetStepper_setCurrentPosition((CPhidgetStepperHandle)SH, 0, 0);
        CPhidgetStepper_setTargetPosition((CPhidgetStepperHandle)SH, 0, directionfactor*this->RADirection*direction*steps);
        this->stopped = false;
    }
    if (g_AllData->getStepperDriverType() == 1) {
        this->sendCommandToAMIS("v",this->speedMax);
        this->sendCommandToAMIS("z");
        this->sendCommandToAMIS("s",this->RADirection*direction*steps);
        this->sendCommandToAMIS("o");
        this->stopped = false;
    }
    qDebug() << "------ Travel initiation done ...";
}

//-----------------------------------------------
void QtContinuousStepper::resetSteppersAfterStop(void) { // this function is called once it was detected that the steppers stopped moving
    this->stopped = true;
    this->speedMax=g_AllData->getCelestialSpeed()*(this->gearRatio)*(this->microsteps);
    if (g_AllData->getStepperDriverType() == 0) {
        CPhidgetStepper_setVelocityLimit((CPhidgetStepperHandle)SH,0,this->speedMax);
    }
    if (g_AllData->getStepperDriverType() == 1) {
        this->sendCommandToAMIS("v",(long)(this->speedMax));
    }
    if (this->isHBoxSlew == true) {
        this->hBoxSlewEnded=true;
        this->isHBoxSlew = false;
    }
}

//-----------------------------------------------
void QtContinuousStepper::setRADirection(short dir) {
    if (abs(dir) == 1) {
        this->RADirection = dir;
    }
}

//-----------------------------------------------

int QtContinuousStepper::retrieveKineticStepperData(int what) {
    int retval = 0;

    if (g_AllData->getStepperDriverType() == 0) {
        switch (what) {
        case 1:
            retval=this->snumifk;
            break;
        case 2:
            retval=this->vifk;
            break;
        default:
            retval=-1;
        }
    }
    return retval;
}

//-----------------------------------------------------------------------------
double QtContinuousStepper::getKineticsFromController(short whichOne) {
    double retval = 0;

    switch (whichOne) {
    case 1:
        if (g_AllData->getStepperDriverType() == 0) {
            CPhidgetStepper_getCurrentLimit((CPhidgetStepperHandle)SH,0,&retval);
        }
        if (g_AllData->getStepperDriverType() == 1) {
            retval = (double)(this->sendCommandToAMIS("f9").toLong()/1000.0);
        }
        break;
    case 2:
        if (g_AllData->getStepperDriverType() == 0) {
            CPhidgetStepper_getAcceleration((CPhidgetStepperHandle)SH,0,&retval);
        }
        if (g_AllData->getStepperDriverType() == 1) {
            retval = (double)(this->sendCommandToAMIS("f8").toLong());
        }
        break;
    case 3:
        if (g_AllData->getStepperDriverType() == 0) {
            CPhidgetStepper_getVelocityLimit((CPhidgetStepperHandle)SH,0,&retval);
        }
        if (g_AllData->getStepperDriverType() == 1) {
            retval = (double)(this->sendCommandToAMIS("f7").toLong());
        }
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

void QtContinuousStepper::setStepperParams(double val, short whichOne) {

    switch (whichOne) {
    case 1:
        this->acc=val;
        if (g_AllData->getStepperDriverType() == 0) {
            CPhidgetStepper_setAcceleration((CPhidgetStepperHandle)SH,0,this->acc);
        }
        if (g_AllData->getStepperDriverType() == 1) {
            this->sendCommandToAMIS("a", this->acc);
        }
        break;
    case 2:
        this->speedMax=val;
        if (g_AllData->getStepperDriverType() == 0) {
            CPhidgetStepper_setVelocityLimit((CPhidgetStepperHandle)SH,0,this->speedMax);
        }
        if (g_AllData->getStepperDriverType() == 1) {
            this->sendCommandToAMIS("v", this->speedMax);
        }
        break;
    case 3:
        if (g_AllData->getStepperDriverType() == 0) {
            if (val > 4) {
                val = 4;
            }
            this->currMax=val;
            CPhidgetStepper_setCurrentLimit((CPhidgetStepperHandle)SH,0,this->currMax);
        }
        if (g_AllData->getStepperDriverType() == 1) {
            if (val > 3) {
                val = 3;
            }
            this->currMax=val;
            this->sendCommandToAMIS("c", this->currMax*1000);
            usleep(100);
        }
    }
    return;
}


//-----------------------------------------------------------------------------

void QtContinuousStepper::shutDownDrive(void) {
    this->stopped=true;
    if (g_AllData->getStepperDriverType() == 0) {
        CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 0);
        sleep(1);
    }
    if (g_AllData->getStepperDriverType() == 1) {
        this->sendCommandToAMIS("x");
        this->sendCommandToAMIS("e",0);
    }
}

//-----------------------------------------------------------------------------

bool QtContinuousStepper::getStopped(void) {
    return (this->stopped);
}

//------------------------------------------------------------------------------

void QtContinuousStepper::stopDrive(void) {

    if (g_AllData->getStepperDriverType() == 0) {
        CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 0);
        CPhidgetStepper_setCurrentPosition((CPhidgetStepperHandle)SH, 0, 0);
        CPhidgetStepper_setTargetPosition((CPhidgetStepperHandle)SH, 0, 0);
        this->stopped = true;
        usleep(100);
    }
    if (g_AllData->getStepperDriverType() == 1) {
        this->sendCommandToAMIS("x");
        this->sendCommandToAMIS("z");
    }
    this->stopped=true;

}

//-------------------------------------------------------------------------------

void QtContinuousStepper::changeSpeedForGearChange(void) {

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
    if (g_AllData->getStepperDriverType() == 0) {
        CPhidgetStepper_setVelocityLimit((CPhidgetStepperHandle)SH,0,this->speedMax);
    }
    if (g_AllData->getStepperDriverType() == 1) {
        this->sendCommandToAMIS("v", this->speedMax);
    }
    // 360°/sidereal day in seconds*gear ratios*microsteps/steps
}

//-------------------------------------------------------------------------------

bool QtContinuousStepper::hasHBoxSlewEnded(void) {
    return this->hBoxSlewEnded;
}

//--------------------------------------------------------------------------------
// two private routines to simplify communications with the AMIS
QString QtContinuousStepper::sendCommandToAMIS(QString cmd, long val) {
    QString theCommand;
    QString theReply;

    theCommand.append(cmd);
    theCommand.append(QString::number(val, 10));
    amisInterface->sendCommand(theCommand,true);
    theReply.append(amisInterface->getReply(true));
  /*  if (cmd != "f7") {
        qDebug() << "--- Command to RA ---";
        qDebug() << "Sent: " << theCommand.toLatin1();
        qDebug() << "Received: " << theReply.toLatin1();
    }
*/    return theReply;
}

//--------------------------------------------------------------------------------
QString QtContinuousStepper::sendCommandToAMIS(QString cmd) {
    QString theCommand;
    QString theReply;

    theCommand.append(cmd);
    amisInterface->sendCommand(theCommand,true);
    theReply.append(amisInterface->getReply(true));
  /*  if (cmd != "f7") {
        qDebug() << "--- Command to RA ---";
        qDebug() << "Sent: " << theCommand.toLatin1();
        qDebug() << "Received: " << theReply.toLatin1();
    } */
    return theReply;
}

