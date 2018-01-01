#include "QtKineticStepper.h"
#include <unistd.h>
#include "tsc_globaldata.h"
#include <stdlib.h>
#include <math.h>
#include <QDebug>

extern TSC_GlobalData *g_AllData;

//-----------------------------------------------------------------------------

QtKineticStepper::QtKineticStepper(void){
    int sernum, version;

    this->SH = NULL;
    this->errorCreate = CPhidgetStepper_create(&SH);
    this->errorOpen = CPhidget_open((CPhidgetHandle)SH, -1);
    sleep(1);
    CPhidget_getSerialNumber((CPhidgetHandle)SH, &sernum);
    CPhidget_getDeviceVersion((CPhidgetHandle)SH, &version);
    sleep(1);
    this->snumifk=sernum;
    this->vifk=version;    
    this->hBoxSlewEnded=false;
    this->stopped=true;
    this->gearRatio = 1;
    this->microsteps = 2;
}

//-----------------------------------------------------------------------------

QtKineticStepper::~QtKineticStepper(void){
    this->stopped=true;
    CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 0);
    sleep(1);
    CPhidget_close((CPhidgetHandle)SH);
    CPhidget_delete((CPhidgetHandle)SH);
}

//-----------------------------------------------------------------------------
void QtKineticStepper::setGearRatioAndMicrosteps(double gr, double ms) {
    this->gearRatio=gr;
    this->microsteps=ms;
}

//-----------------------------------------------------------------------------

void QtKineticStepper::setInitialParamsAndComputeBaseSpeed(double lacc, double lcurr) {
    double smax, amax, smin;

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
        this->currMax=lcurr;
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

//-----------------------------------------------------------------------------
bool QtKineticStepper::travelForNSteps(long steps,short direction, int factor,bool isHBSlew) {

    this->hBoxSlewEnded = false;
    this->speedMax=factor*g_AllData->getCelestialSpeed()*(this->gearRatio)*(this->microsteps);
    if (direction < 0) {
        direction = -1;
    } else {
        direction = 1;
    }
    CPhidgetStepper_setVelocityLimit((CPhidgetStepperHandle)SH,0,this->speedMax);
    CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 1);
    CPhidgetStepper_setCurrentPosition((CPhidgetStepperHandle)SH, 0, 0);
    CPhidgetStepper_setTargetPosition((CPhidgetStepperHandle)SH, 0, direction*steps);
    this->stopped = false;
    while (this->stopped == false) {
        CPhidgetStepper_getStopped((CPhidgetStepperHandle)SH, 0, &stopped);
    }
    CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 0);
    this->speedMax=g_AllData->getCelestialSpeed()*(this->gearRatio)*(this->microsteps);
    CPhidgetStepper_setVelocityLimit((CPhidgetStepperHandle)SH,0,this->speedMax);
    if (isHBSlew == 1) {
        this->hBoxSlewEnded=true;
    }
    return true;
}

//-----------------------------------------------------------------------------

int QtKineticStepper::retrieveKineticStepperData(int what) {
    int retval;

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
    return retval;
}

//-----------------------------------------------------------------------------

double QtKineticStepper::getKineticsFromController(short whichOne) {
    double retval;

    switch (whichOne) {
    case 1:
        CPhidgetStepper_getCurrentLimit((CPhidgetStepperHandle)SH,0,&retval);
        break;
    case 2:
        CPhidgetStepper_getAcceleration((CPhidgetStepperHandle)SH,0,&retval);
        break;
    case 3:
        CPhidgetStepper_getVelocityLimit((CPhidgetStepperHandle)SH,0,&retval);
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
        CPhidgetStepper_setAcceleration((CPhidgetStepperHandle)SH,0,this->acc);
        break;
    case 2:
        this->speedMax=val;
        CPhidgetStepper_setVelocityLimit((CPhidgetStepperHandle)SH,0,this->speedMax);
        break;
    case 3:
        if (val > 4) {
            val = 4;
        }
        this->currMax=val;
        CPhidgetStepper_setCurrentLimit((CPhidgetStepperHandle)SH,0,this->currMax);
    }
    return;
}


//-----------------------------------------------------------------------------

void QtKineticStepper::shutDownDrive(void) {

    this->stopped=true;
    CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 0);
    sleep(1);
}

//-----------------------------------------------------------------------------

bool QtKineticStepper::getStopped(void) {
    return (this->stopped);
}

//------------------------------------------------------------------------------

void QtKineticStepper::stopDrive(void) {

    CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 0);
    CPhidgetStepper_setCurrentPosition((CPhidgetStepperHandle)SH, 0, 0);
    CPhidgetStepper_setTargetPosition((CPhidgetStepperHandle)SH, 0, 0);
    this->stopped=true;
    usleep(100);
}

//-------------------------------------------------------------------------------

void QtKineticStepper::engageDrive(void) {
    this->stopped=false;
    CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 1);
}

//-------------------------------------------------------------------------------

void QtKineticStepper::changeSpeedForGearChange(void) {

    this->stepsPerSecond=round(g_AllData->getCelestialSpeed()*(this->gearRatio)*(this->microsteps));
    if (this->stepsPerSecond < 10) {
        this->speedMax = 10; // a speed of less than 10 microsteps is not achievable - it is physics, baby ...
        this->stepsPerSecond = this->speedMax;
    }
    if (this->stepsPerSecond > 5000) {
        this->speedMax = 50000; // a speed of more than 50000 microsteps is not achievable - it is physics, baby ...
        this->stepsPerSecond = this->speedMax;
    }
    this->speedMax=stepsPerSecond;
    CPhidgetStepper_setVelocityLimit((CPhidgetStepperHandle)SH,0,this->speedMax);
    // 360°/sidereal day in seconds*gear ratios*microsteps/steps
}

//-------------------------------------------------------------------------------

bool QtKineticStepper::hasHBoxSlewEnded(void) {
    return this->hBoxSlewEnded;
}

