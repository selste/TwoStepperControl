#ifndef QSTEPPERPHIDGETSRA_H
#define QSTEPPERPHIDGETSRA_H

#include <phidget21.h>

class QStepperPhidgetsRA {
private:
    CPhidgetStepperHandle SH;
    int errorOpen;
    int errorCreate;
    int snumifk;
    int vifk;
    int motorNum;
    double speedMax;
    double speedMin;
    double acc;
    double currMax;
    int stopped;
    double stepsPerSInRA;
    short RADirection;
    bool hBoxSlewEnded;

public:
    QStepperPhidgetsRA(double,double);
    ~QStepperPhidgetsRA(void);
    void startTracking(void);
    void setRADirection(short);
    bool travelForNSteps(long,short,int,bool);
    int retrievePhidgetStepperData (int);
    double getKinetics(short);
    void setStepperParams(double, short);
    void shutDownDrive(void);
    bool getStopped(void);
    void stopDrive(void);
    void engageDrive(void);
    void changeSpeedForGearChange(void);
    bool hasHBoxSlewEnded(void);
};
#endif // QSTEPPERPHIDGETSRA_H
