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
    this->speedMax=smax/500;
    this->speedMin=smin;
    this->accMax=amax/500;
    this->currMax=2.5;
    CPhidgetStepper_setCurrentLimit((CPhidgetStepperHandle)SH,0,currMax);

    this->RATrackingTimer = new QElapsedTimer();
    this->stepsPerMSInRA=0.0041780746*
            (g_AllData->getGearData(0))*
            (g_AllData->getGearData(1))*
            (g_AllData->getGearData(2))*
            (g_AllData->getGearData(8))/(g_AllData->getGearData(3))/1000.0;
    qDebug() << "microsteps per second" << stepsPerMSInRA;
    // 360°/sidereal day in seconds*gear ratios*microsteps/steps in ms
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

void QStepperPhidgets::startTracking(qint64 timeElapsed) {
    long stepsAhead;

    stepsAhead=round(timeElapsed*this->stepsPerMSInRA);
    qDebug() << stepsAhead;

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

void QStepperPhidgets::setDriveToStopped(bool isStop) {
    this->stopped=isStop;
}

//-----------------------------------------------------------------------------

void QStepperPhidgets::setTrackingOnOff(bool isOn) {
    this->setTrackingOnOff(isOn);
}
