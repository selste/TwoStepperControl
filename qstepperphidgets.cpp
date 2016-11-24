#include "qstepperphidgets.h"
#include <unistd.h>
#include <stdlib.h>
#include "tsc_globaldata.h"
#include <stdlib.h>
#include <math.h>
#include <QDebug>

extern TSC_GlobalData *g_AllData;

//-----------------------------------------------------------------------------

QStepperPhidgets::QStepperPhidgets(void){
    int sernum, version, numMotors;
    double smax, amax, smin;

    this->SH = NULL;
    this->errorCreate = CPhidgetStepper_create(&SH);
    this->errorOpen = CPhidget_open((CPhidgetHandle)SH, -1);
    sleep(2);
    CPhidget_getSerialNumber((CPhidgetHandle)SH, &sernum);

    CPhidget_getDeviceVersion((CPhidgetHandle)SH, &version);
    CPhidgetStepper_getMotorCount (SH, &numMotors);
    sleep(2);
    this->snumifk=sernum;
    this->vifk=version;
    this->motorNum=numMotors;
    CPhidgetStepper_getVelocityMax((CPhidgetStepperHandle)SH,0,&smax);
    CPhidgetStepper_getVelocityMin((CPhidgetStepperHandle)SH,0,&smin);
    CPhidgetStepper_getAccelerationMax((CPhidgetStepperHandle)SH,0,&amax);
    this->speedMin=smin;
    this->accMax=1000;
    this->currMax=1;
    CPhidgetStepper_setAcceleration((CPhidgetStepperHandle)SH,0,this->accMax);
    CPhidgetStepper_setCurrentLimit((CPhidgetStepperHandle)SH,0,currMax);
    this->stopped=true;

    this->stepsPerSInRA=0.0041780746*
            (g_AllData->getGearData(0))*
            (g_AllData->getGearData(1))*
            (g_AllData->getGearData(2))*
            (g_AllData->getGearData(8))/(g_AllData->getGearData(3));
    this->speedMax=stepsPerSInRA;
    CPhidgetStepper_setVelocityLimit((CPhidgetStepperHandle)SH,0,this->speedMax);
    // 360°/sidereal day in seconds*gear ratios*microsteps/steps
}

//-----------------------------------------------------------------------------

QStepperPhidgets::~QStepperPhidgets(void){
    this->stopped=PTRUE;
    CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 0);
    sleep(1);
    CPhidget_close((CPhidgetHandle)SH);
    CPhidget_delete((CPhidgetHandle)SH);
}

//-----------------------------------------------------------------------------

void QStepperPhidgets::startTracking(void) {
    CPhidgetStepper_setVelocityLimit((CPhidgetStepperHandle)SH,0,this->speedMax);
    CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 1);
    CPhidgetStepper_setCurrentPosition((CPhidgetStepperHandle)SH, 0, 0);
    CPhidgetStepper_setTargetPosition((CPhidgetStepperHandle)SH, 0, (60*60*24*this->stepsPerSInRA));
    // one day is maximum tracking time
    this->stopped = false;
    while (this->stopped == false) {
        CPhidgetStepper_getStopped((CPhidgetStepperHandle)SH, 0, &stopped);
    }
    CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 0);
}

//-----------------------------------------------------------------------------
bool QStepperPhidgets::travelForNSteps(long steps,short direction, int factor) {
    if (direction < 0) {
        direction = -1;
    } else {
        direction = 1;
    }
    CPhidgetStepper_setVelocityLimit((CPhidgetStepperHandle)SH,0,this->speedMax);
    CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 1);
    CPhidgetStepper_setCurrentPosition((CPhidgetStepperHandle)SH, 0, 0);
    CPhidgetStepper_setTargetPosition((CPhidgetStepperHandle)SH, 0, factor*direction*steps);
    this->stopped = false;
    while (this->stopped == false) {
        CPhidgetStepper_getStopped((CPhidgetStepperHandle)SH, 0, &stopped);
    }
    CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 0);
    return true;
}

//-----------------------------------------------------------------------------
int QStepperPhidgets::retrievePhidgetStepperData(int what) {
    int retval;

    switch (what) {
    case 1:
        retval=this->snumifk;
        break;
    case 2:
        retval=this->vifk;
        break;
    case 3:
        retval=this->motorNum;
        break;
    default:
        retval=-1;
    }
    return retval;
}

//-----------------------------------------------------------------------------

double QStepperPhidgets::getKinetics(short whichOne) {
    double retval;

    switch (whichOne) {
    case 1:
        retval = (this->currMax);
        break;
    case 2:
        retval = (this->accMax);
        break;
    case 3:
        retval = (this->speedMax);
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


void QStepperPhidgets::setKinetics(double val, short whichOne) {

    switch (whichOne) {
    case 1:
        this->accMax=val;
        break;
    case 2:
        this->speedMax=val;
        break;
    }
    return;
}

//-----------------------------------------------------------------------------

void QStepperPhidgets::shutDownDrive(void) {

    this->stopped=PTRUE;
    CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 0);
    sleep(1);
}

//-----------------------------------------------------------------------------

void QStepperPhidgets::setStopped(bool isStop) {
    this->stopped=isStop;
}

//-----------------------------------------------------------------------------

bool QStepperPhidgets::getStopped(void) {
    return (this->stopped);
}

//------------------------------------------------------------------------------

void QStepperPhidgets::stopDrive(void) {
    this->stopped=true;
    CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 0);
    CPhidgetStepper_setCurrentPosition((CPhidgetStepperHandle)SH, 0, 0);
    CPhidgetStepper_setTargetPosition((CPhidgetStepperHandle)SH, 0, 0);
}

//-------------------------------------------------------------------------------

void QStepperPhidgets::engageDrive(void) {
    this->stopped=false;
    CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 1);
}
