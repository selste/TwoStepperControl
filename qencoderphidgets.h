#ifndef QENCODERPHIDGETS_H
#define QENCODERPHIDGETS_H

#include <phidget21.h>

class QEncoderPhidgets {
private:
    CPhidgetEncoderHandle EH;
    int errorOpen;
    int errorCreate;
    int snumifk;
    int vifk;
    int EncoderNum;
    int InputNum;
    float calFactor;
public:
    QEncoderPhidgets(void);
    ~QEncoderPhidgets(void);
    int getTopicalReadingFromEncoder(void);
    int retrievePhidgetEncoderData (int);
    void resetEncoder(void);
    float getCalibrationFactor(void);
    void setCalibrationFactor(float);
};



#endif // QENCODERPHIDGETS_H
