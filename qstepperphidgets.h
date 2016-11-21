#ifndef QSTEPPERPHIDGETS_H
#define QSTEPPERPHIDGETS_H

#include <phidget21.h>
#include <QElapsedTimer>

class QStepperPhidgets {
private:
    QElapsedTimer *RATrackingTimer; // this one must not be reset
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
    double stepsPerMSInRA;

public:
    QStepperPhidgets(void);
    ~QStepperPhidgets(void);
    void startTracking(qint64);
    int retrievePhidgetStepperData (int);
    double getKinetics(short);
    void setKinetics(double, short);
    void shutDownDrive(void);
    void setDriveToStopped(bool);
    void setTrackingOnOff(bool);
};
#endif // QSTEPPERPHIDGETS_H
