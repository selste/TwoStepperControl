// a child class of QtKineticStepper; this one does, in addition, carry a permanent motion. in TSC, this is the right ascension drive ...
#ifndef QTCONTINUOUSSTEPPER_H
#define QTCONTINUOUSSTEPPER_H

#include "QtKineticStepper.h"

class QtContinuousStepper:public QtKineticStepper {
protected:
    short RADirection = 1; // a value that takes +/-1; it inverts continuous motion, for instance when moving to the southern hemisphere
public:
    void startTracking(void); // start continuous motion to compensate for earth's rotation
    bool travelForNSteps(long,short,int,bool) override; // override this function from "QtKineticStepper" so that "RADirection" is also respected ...
    void setRADirection(short); // switch "RADirection"
};
#endif // QTCONTINUOUSSTEPPER_H
