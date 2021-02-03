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
// a base class handling stepper motors that are controlled by accceleration and speed (=kinetic parameters);
// it is a parent to QtContinuousStepper, which features motors that do not only carry out motion on demand,
// but also a permamnent motion on top of it. for TSC, this class is parent to the declination drive, whereas
// the RA drive is a continuous drive ...

#ifndef QTKINETICSTEPPER_H
#define QTKINETICSTEPPER_H

#include <QString>
//#include "usb_communications.h"

class QtKineticStepper {

private:
    int errorOpen; // error received when opening a communication channel
    int errorCreate; // error received when creating a contact to the controller of the drive
    int snumifk; // an identifier for the controller, can be a serial number or something similar
    int vifk; // version number of the interface
    double speedMax; // maximum speed in microsteps/sec
    double speedMin; // minimal speed in microsteps per second
    double acc; // acceleration value for the stepper in microsteps/s^2
    double currMax; // maximum current if it can be set via the software
    double gearRatio; // the product of planetary gear, intermediated gear and worm wheels divided by step size
    double microsteps; // the current number of microsteps
    int stopped; // a flag that indicates whether the drive has stopped
    double stepsPerSecond; // the current rate of microsteps per second
    bool hBoxSlewEnded; // a boolean that is set to true when a long slew has timed out; needed for the handbox-slew from TSC
    bool isHBoxSlew;
    QString sendCommandToAMIS(QString, long);
    QString sendCommandToAMIS(QString);

public:
    QtKineticStepper(void); // contructor, gets maximum acceleration and maximum current
    ~QtKineticStepper(void);
    void setGearRatioAndMicrosteps(double, double); // the product of the gears divided by the step size and the number of microsteps is stored here
    void changeMicroSteps(double); // switches the microstepping ratio for variable drivers
    void setInitialParamsAndComputeBaseSpeed(double,double); // after opening
    void travelForNSteps(long,short,int,bool); // tell the drive to travel for steps, direction (+/-1),
        // a multiple of sidereal speed and a flag that indicates whether the slew was triggered by the handbox.
        // handbox slews terminate either after 180 or 360 degrees ...
    void travelForNSteps(short,float); // tell the drive to travel a constant number of steps in direction (+/-1) and a fraction of sidereal speed - used in ST4 guiding
    double getKineticsFromController(short); //get parameters from controller such as maximum current, currently set acceleration, currently set velocity and so on ...
    bool getErrorFromDriver(void); // return the state of the error pin
    void setStepperParams(double, short); // set acceleration, speed and current and convey it to the controller
    void shutDownDrive(void); // set motor to "unengaged state" - no more current is applied
    bool getStopped(void); // check whether the motor is active or not ...
    void stopDrive(void); // halt the motor
    void resetSteppersAfterStop(void);
    void hwResetDriver(void);
    //void setDriveToStopped(void); // necessary to convey the AMIS that they were stopped
    //void engageDrive(void); // set motor to "engaged" stae without driving it
    void changeSpeedForGearChange(void); // a callback that changes speeds if the gear ratios change
    bool hasHBoxSlewEnded(void); // retrieve the state of the "hBoxSlewEnded" - flag ...
};
#endif // QTKINETICSTEPPER_H
