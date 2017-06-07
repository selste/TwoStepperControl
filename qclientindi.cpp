#include <string>
#include <iostream>
#include <fstream>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <memory>
#include <sys/types.h>
#include <sys/stat.h>
#include "libindi/config.h"
#include "indibase/baseclient.h"
#include "indibase/basedevice.h"
#include "indibase/indiproperty.h"

/* INDI Common Library Routines */
#include "indicom.h"
#include "qclientindi.h"

using namespace std;
#define MYCCD "Simple CCD"

//--------------------------------------------------------
qclientindi::qclientindi() {
    ccd_simulator = NULL;
}

//--------------------------------------------------------
qclientindi::~qclientindi() {

}

//--------------------------------------------------------
void qclientindi::setTemperature() {

   INumberVectorProperty *ccd_temperature = NULL;
   ccd_temperature = ccd_simulator->getNumber("CCD_TEMPERATURE");
   if (ccd_temperature == NULL) {
       IDLog("Error: unable to find CCD Simulator CCD_TEMPERATURE property...\n");
       return;
   }
   ccd_temperature->np[0].value = -20;
   sendNewNumber(ccd_temperature);
}

//--------------------------------------------------------
void qclientindi::takeExposure() {

    INumberVectorProperty *ccd_exposure = NULL;
    ccd_exposure = ccd_simulator->getNumber("CCD_EXPOSURE");
    if (ccd_exposure == NULL)     {
        IDLog("Error: unable to find CCD Simulator CCD_EXPOSURE property...\n");
        return;
    }

    // Take a 1 second exposure
    IDLog("Taking a 1 second exposure.\n");
    ccd_exposure->np[0].value = 1;
    sendNewNumber(ccd_exposure);
}

//--------------------------------------------------------
void qclientindi::newDevice(INDI::BaseDevice *dp) {

    if (!strcmp(dp->getDeviceName(), MYCCD))
        IDLog("Receiving %s Device...\n", dp->getDeviceName());

    ccd_simulator = dp;
}

//--------------------------------------------------------
void qclientindi::newProperty(INDI::Property *property) {

    if (!strcmp(property->getDeviceName(), MYCCD) && !strcmp(property->getName(), "CONNECTION")) {
        connectDevice(MYCCD);
        return;
    }

    if (!strcmp(property->getDeviceName(), MYCCD) && !strcmp(property->getName(), "CCD_TEMPERATURE")) {
        if (ccd_simulator->isConnected()) {
            IDLog("CCD is connected. Setting temperature to -20 C.\n");
            setTemperature();
        }
        return;
    }
}

//--------------------------------------------------------
void qclientindi::newNumber(INumberVectorProperty *nvp) {
    // Let's check if we get any new values for CCD_TEMPERATURE
    if (!strcmp(nvp->name, "CCD_TEMPERATURE"))
    {
       IDLog("Receving new CCD Temperature: %g C\n", nvp->np[0].value);

       if (nvp->np[0].value == -20)        {
           IDLog("CCD temperature reached desired value!\n");
           takeExposure();
       }
   }
}

//--------------------------------------------------------
void qclientindi::newMessage(INDI::BaseDevice *dp, int messageID) {

     if (strcmp(dp->getDeviceName(), MYCCD))
         return;

     IDLog("Recveing message from Server:\n\n########################\n%s\n########################\n\n", dp->messageQueue(messageID).c_str());
}

//--------------------------------------------------------
void qclientindi::newBLOB(IBLOB *bp) {
    // Save FITS file to disk
    ofstream myfile;
    myfile.open ("ccd_simulator.fits", ios::out | ios::binary);

    myfile.write(static_cast<char *> (bp->blob), bp->bloblen);

    myfile.close();

    IDLog("Received image, saved as ccd_simulator.fits\n");
}

//--------------------------------------------------------


