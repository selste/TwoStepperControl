#include "qstepperphidgets.h"
#include <unistd.h>
#include <stdlib.h>


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
    this->speedMax=10000;
    this->speedMin=smin;
    this->accMax=100000;
    this->currAccMax=accMax;
    this->currSpeedMax=speedMax;
    this->currSpeedMin=speedMin;
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

int QStepperPhidgets::sendSteps(int noOfSteps) {
    long long int stepsDone;
    int iSD = 0;

    CPhidgetStepper_setAcceleration((CPhidgetStepperHandle)SH,0,this->currAccMax);
    CPhidgetStepper_setVelocityLimit((CPhidgetStepperHandle)SH,0,this->currSpeedMax);
    CPhidgetStepper_setCurrentLimit((CPhidgetStepperHandle)SH,0,1);
    CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 1);
    CPhidgetStepper_setCurrentPosition((CPhidgetStepperHandle)SH, 0, 0);
    CPhidgetStepper_setTargetPosition((CPhidgetStepperHandle)SH, 0, noOfSteps);
    this->stopped = PFALSE;
    while (this->stopped == PFALSE) {
        CPhidgetStepper_getStopped((CPhidgetStepperHandle)SH, 0, &stopped);
    }
    CPhidgetStepper_getCurrentPosition((CPhidgetStepperHandle)SH, 0, &stepsDone);
    CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 0);
    iSD=(int)stepsDone;
    return iSD;
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

void QStepperPhidgets::setAcc(double acc) {
    if ((acc < this->accMax) && (acc > 500)) {
        this->currAccMax=acc;
    }
}

//-----------------------------------------------------------------------------

double QStepperPhidgets::getKinetics(short whichOne) {
    double retval;

    switch (whichOne) {
    case 1:
        retval = (this->currAccMax);
        break;
    case 2:
        retval = (this->accMax);
        break;
    case 3:
        retval = (this->currSpeedMax);
        break;
    case 4:
        retval = (this->currSpeedMin);
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
        this->currAccMax=val;
        break;
    case 2:
        this->currSpeedMax=val;
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
