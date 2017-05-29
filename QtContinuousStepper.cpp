#include "QtContinuousStepper.h"
#include <stdlib.h>
#include <unistd.h>
#include "tsc_globaldata.h"
#include <QDebug>

extern TSC_GlobalData *g_AllData;

//----------------------------------------------

void QtContinuousStepper::startTracking(void) {
    this->speedMax=g_AllData->getCelestialSpeed()*(this->gearRatio*this->microsteps);
    CPhidgetStepper_setVelocityLimit((CPhidgetStepperHandle)SH,0,this->speedMax);
    CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 1);
    CPhidgetStepper_setCurrentPosition((CPhidgetStepperHandle)SH, 0, 0);
    CPhidgetStepper_setTargetPosition((CPhidgetStepperHandle)SH, 0, this->RADirection*(60*60*24*this->stepsPerSecond));
    // one day is maximum tracking time
    this->stopped = false;
    while (this->stopped == false) {
        CPhidgetStepper_getStopped((CPhidgetStepperHandle)SH, 0, &stopped);
    }
    CPhidgetStepper_setEngaged((CPhidgetStepperHandle)SH, 0, 0);
}

//-----------------------------------------------
bool QtContinuousStepper::travelForNSteps(long steps,short direction, int factor, bool isHBSlew) {

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
    CPhidgetStepper_setTargetPosition((CPhidgetStepperHandle)SH, 0, this->RADirection*direction*steps);
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

//-----------------------------------------------
void QtContinuousStepper::setRADirection(short dir) {
    if (abs(dir) == 1) {
        this->RADirection = dir;
    }
}

//-----------------------------------------------


