// a base class handling stepper motors that are controlled by accceleration and speed (=kinetic parameters);
// it is a parent to QtContinuousStepper, which features motors that do not only carry out motion on demand,
// but also a permamnent motion on top of it. for TSC, this class is parent to the declination drive, whereas
// the RA drive is a continuous drive ...

#ifndef QTKINETICSTEPPER_H
#define QTKINETICSTEPPER_H

#include <phidget21.h>

class QtKineticStepper {
protected:
    CPhidgetStepperHandle SH;
    int errorOpen; // error received when opening a communication channel
    int errorCreate; // error received when creating a contact to the controller of the drive
    int snumifk; // an identifier for the controller, can be a serial number or something similar
    int vifk; // version number of the interface
    double speedMax; // maxiumum speed in microsteps/sec
    double speedMin; // minimal speed in microsteps per second
    double acc; // acceleration value for the stepper in microsteps/s^2
    double currMax; // maximum current if it can be set via the software
    double gearRatio; // the product of planetary gear, intermediated gear and worm wheels divided by step size
    double microsteps; // the current number of microsteps
    int stopped; // a flag that indicates whether the drive has stopped
    double stepsPerSecond; // the current rate of microsteps per second
    bool hBoxSlewEnded; // a boolean that is set to true when a long slew has timed out; needed for the handbox-slew from TSC

public:
    QtKineticStepper(void); // contructor, gets maximum acceleration and maximum current
    virtual ~QtKineticStepper(void); // destructor
    void setGearRatioAndMicrosteps(double, double); // the product of the gears divided by the step size and the number of microsteps is stored here
    void setInitialParamsAndComputeBaseSpeed(double,double); // after opening
    virtual bool travelForNSteps(long,short,int,bool); // tell the drive to travel for steps, direction (+/-1),
        // a multiple of sidereal speed and a flag that indicates whether the slew was triggered by the handbox.
        // handbox slews terminate either after 180 or 360 degrees ...
    int retrieveKineticStepperData (int); // retrieve basic controller data such as identifiers of the controller
    double getKineticsFromController(short); //get parameters from controller such as maximum current, currently set acceleration, currently set velocity and so on ...
    void setStepperParams(double, short); // set acceleration, speed and current and convey it to the controller
    void shutDownDrive(void); // set motor to "unengaged state" - no more current is applied
    bool getStopped(void); // check whether the motor is active or not ...
    void stopDrive(void); // halt the motor
    void engageDrive(void); // set motor to "engaged" stae without driving it
    void changeSpeedForGearChange(void); // a callback that changes speeds if the gear ratios change
    bool hasHBoxSlewEnded(void); // retrieve the state of the "hBoxSlewEnded" - flag ...
};
#endif // QTKINETICSTEPPER_H
