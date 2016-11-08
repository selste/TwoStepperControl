#include <phidget21.h>
#include "qencoderphidgets.h"
#include <unistd.h>
#include <stdlib.h>


//-----------------------------------------------------------------------------

QEncoderPhidgets::QEncoderPhidgets(void){
    int sernum, version, numEncs, numInps;

    this->EH = NULL;
    this->errorCreate = CPhidgetEncoder_create(&EH);
    this->errorOpen = CPhidget_open((CPhidgetHandle)EH, -1);
    sleep(1);
    CPhidgetEncoder_setEnabled((CPhidgetEncoderHandle)EH,0,1);
    CPhidget_getSerialNumber((CPhidgetHandle)EH, &sernum);
    CPhidget_getDeviceVersion((CPhidgetHandle)EH, &version);
    CPhidgetEncoder_getEncoderCount (EH, &numEncs);
    CPhidgetEncoder_getInputCount (EH, &numInps);
    this->snumifk=sernum;
    this->vifk=version;
    this->EncoderNum=numEncs;
    this->InputNum=numInps;
    this->calFactor=2.222222222;
}

//-----------------------------------------------------------------------------

QEncoderPhidgets::~QEncoderPhidgets(void){
    CPhidgetEncoder_setEnabled((CPhidgetEncoderHandle)EH,0,0);
    CPhidget_close((CPhidgetHandle)EH);
    CPhidget_delete((CPhidgetHandle)EH);
    sleep(1);
}

//-----------------------------------------------------------------------------

int QEncoderPhidgets::getTopicalReadingFromEncoder(void) {
    int stepsDone;

    CPhidgetEncoder_getPosition(EH, 0, &stepsDone);
    return stepsDone;
}

//-----------------------------------------------------------------------------

void QEncoderPhidgets::resetEncoder(void) {
    CPhidgetEncoder_setPosition(EH, 0, 0);
}

//-----------------------------------------------------------------------------

int QEncoderPhidgets::retrievePhidgetEncoderData(int what) {
    int retval;

    switch (what) {
    case 1:
        retval=this->snumifk;
        break;
    case 2:
        retval=this->vifk;
        break;
    case 3:
        retval=this->EncoderNum;
        break;
    case 4:
        retval=this->InputNum;
        break;
    default:
        retval=-1;
    }
    return retval;
}

//-----------------------------------------------------------------------------


void QEncoderPhidgets::setCalibrationFactor(float cf) {
    this->calFactor = cf;
}

//-----------------------------------------------------------------------------

float QEncoderPhidgets::getCalibrationFactor(void) {
    return calFactor;
}

