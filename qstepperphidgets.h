#ifndef QSTEPPERPHIDGETS_H
#define QSTEPPERPHIDGETS_H

#include <phidget21.h>

class QStepperPhidgets {
private:
    CPhidgetStepperHandle SH;
    int errorOpen;
    int errorCreate;
    int snumifk;
    int vifk;
    int motorNum;
    double speedMax;
    double speedMin;
    double accMax;
    double currMax;
    int stopped;
    double stepsPerSInRA;

public:
    QStepperPhidgets(void);
    ~QStepperPhidgets(void);
    void startTracking(void);
    bool travelForNSteps(long,short,int);
    int retrievePhidgetStepperData (int);
    double getKinetics(short);
    void setStepperParams(double, short);
    void shutDownDrive(void);
    void setStopped(bool);
    bool getStopped(void);
    void stopDrive(void);
    void engageDrive(void);
};
#endif // QSTEPPERPHIDGETS_H
