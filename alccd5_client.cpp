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
#include <config.h>
#include <qdebug.h>
#include <qcolor.h>
#include <qpixmap.h>

#include "indibase/baseclient.h"
#include "indibase/basedevice.h"
#include "indibase/indiproperty.h"
/* INDI Common Library Routines */
#include "indicom.h"

#include "alccd5_client.h"

using namespace std;

#define MYCCD "QHY CCD QHY5-0-M-"

//------------------------------------------
alccd5_client::alccd5_client() {
    alccd5 = NULL;
    fitsqimage = NULL;
    displayPMap = new QPixmap();
    this->imgwidth = 1280;
    this->imgheight = 1024;
    newCameraImageAvailable = false;
}

//------------------------------------------
alccd5_client::~alccd5_client() {
}

//------------------------------------------
bool alccd5_client::setINDIServer(QString addr, int port) {
    bool serverconnected;

    QByteArray ba = addr.toLatin1();
    const char *c_str2 = ba.data();
    this->setServer(c_str2, port);
    this->watchDevice(MYCCD);
    serverconnected= this->connectServer();
    this->setBLOBMode(B_ALSO, MYCCD, NULL);
    return serverconnected;
}

//------------------------------------------
void alccd5_client::takeExposure(int expTime) {
    INumberVectorProperty *ccd_exposure = NULL;

    if (alccd5->isConnected()) {
        ccd_exposure = alccd5->getNumber("CCD_EXPOSURE");
        if (ccd_exposure == NULL)     {
            qDebug() << "Error: unable to find CCD Simulator CCD_EXPOSURE property...";
            return;
        }
        newCameraImageAvailable = false;
        // set flag to false as acquisition is under way
        ccd_exposure->np[0].value = expTime;
        sendNewNumber(ccd_exposure);
    } else {
        qDebug() << "Cam not connected...";
    }
}

//------------------------------------------
void alccd5_client::newDevice(INDI::BaseDevice *dp) {
    if (!strcmp(dp->getDeviceName(), MYCCD))
        qDebug() << "Receiving Device:" << dp->getDeviceName();
    alccd5 = dp;
}

//------------------------------------------
void alccd5_client::newProperty(INDI::Property *property) {
    if (!strcmp(property->getDeviceName(), MYCCD) && !strcmp(property->getName(), "CONNECTION")) {
        connectDevice(MYCCD);
        return;
    }
      return;
}

//------------------------------------------
void alccd5_client::newNumber(INumberVectorProperty *nvp) {

}

//------------------------------------------
void alccd5_client::newMessage(INDI::BaseDevice *dp, int messageID) {
     if (strcmp(dp->getDeviceName(), MYCCD))
         return;
     qDebug() << "Recveing message from Server: " << dp->messageQueue(messageID).c_str();
}

//------------------------------------------
void alccd5_client::newBLOB(IBLOB *bp) {
    // receive a FITS file from the server ...
    // header is 2880, image is 1280x1024
    char *fitsdata;
    QImage *smallQImage, *mimage;
    ofstream fitsfile;

    fitsdata = static_cast<char *>(bp->blob);
    // casting the INDI-BLOB containing the image data in FITS to an array of uints ...

    fitsdata=fitsdata+2880;
    // skipping the header which is 2880 bytes for the fits from the QHY5

    QVector<QRgb> colorTable(256);
    for(int i=0;i<256;i++) {
        colorTable[i] = qRgb(i,i,i);
    }
    // setting colortable for grayscale QImages

    fitsqimage = new QImage((uchar*)fitsdata, this->imgwidth,this->imgheight, QImage::Format_Indexed8);
    fitsqimage->setColorTable(colorTable);
    mimage = new QImage(fitsqimage->mirrored(0,1));
    // read the image data into a QImage, set a grayscale LUT, and mirror the image ...

//    mimage->save("TestCameraImage.jpg",0,-1);
    // uncomment if you want to see the QImage ...
    smallQImage = new QImage(mimage->scaled(225,180,Qt::KeepAspectRatio,Qt::FastTransformation));
    displayPMap->convertFromImage(*smallQImage,0);
    delete smallQImage;
    delete mimage;
    newCameraImageAvailable = true;

//    fitsfile.open ("alccd5.fits", ios::out | ios::binary);
//    fitsfile.write(static_cast<char *> (bp->blob), bp->bloblen);
//    fitsfile.close();
    // uncomment to save the FITS-file from the INDI-server ...
}

//------------------------------------------

QPixmap* alccd5_client::getScaledPixmapFromCamera(void) {
    // deliver a small image from the camera to other routines for display
    return displayPMap;
}

//------------------------------------------

bool alccd5_client::newImageArrived(void) {
    // return the state whether image acquisition form the camera was completed
    return newCameraImageAvailable;
}

//------------------------------------------

void alccd5_client::newImageUsedAsPixmap(void) {
    // allow other classes to set the state to flase if the pixmap was already used
    newCameraImageAvailable = false;
}

//------------------------------------------
