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
    double currAccMax;
    double currSpeedMax;
    double currSpeedMin;
    int stopped;
public:
    QStepperPhidgets(void);
    ~QStepperPhidgets(void);
    int sendSteps(int);
    int retrievePhidgetStepperData (int);
    void setAcc(double);
    void resetAcc(void);
    double getKinetics(short);
    void setKinetics(double, short);
    void shutDownDrive(void);
};
#endif // QSTEPPERPHIDGETS_H
