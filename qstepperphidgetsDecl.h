#ifndef QSTEPPERPHIDGETSDECL_H
#define QSTEPPERPHIDGETSDECL_H

#include <phidget21.h>

class QStepperPhidgetsDecl {
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
    double stepsPerSInDecl;

public:
    QStepperPhidgetsDecl(double,double);
    ~QStepperPhidgetsDecl(void);
    bool travelForNSteps(long,short,int);
    int retrievePhidgetStepperData (int);
    double getKinetics(short);
    void setStepperParams(double, short);
    void shutDownDrive(void);
    bool getStopped(void);
    void stopDrive(void);
    void engageDrive(void);
    void changeSpeedForGearChange(void);
};
#endif // QSTEPPERPHIDGETSDECL_H
