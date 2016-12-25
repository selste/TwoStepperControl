// derived from INDI client example; currently, it connects only to the QHY5

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
#include "indicom.h"

#include "tsc_globaldata.h"
#include "alccd5_client.h"

using namespace std;

#define MYCCD "QHY CCD QHY5-0-M-"

extern TSC_GlobalData *g_AllData;

//------------------------------------------
alccd5_client::alccd5_client() {
    QRgb cval;

    this->alccd5 = NULL;
    this->fitsqimage = NULL;
    this->displayPMap = new QPixmap();
    this->serverMessage= new QString();
    this->myVec =new QVector<QRgb>(256);
    for(int i=0;i<256;i++) {
        cval = qRgb(i,i,i);
        this->myVec->insert(i, cval);
    }
    // setting colortable for grayscale QImages
}

//------------------------------------------
alccd5_client::~alccd5_client() {
    delete fitsqimage;
    delete displayPMap;
    delete myVec;
    delete serverMessage;
}

//------------------------------------------
bool alccd5_client::setINDIServer(QString addr, int port) {
    bool serverconnected;

    QByteArray ba = addr.toLatin1();
    const char *c_str2 = ba.data();
    this->setServer(c_str2, port);
    this->watchDevice(MYCCD);
    serverconnected= this->connectServer();
    if (serverconnected==true) {
        this->setBLOBMode(B_ALSO, MYCCD, NULL);
    }
    return serverconnected;
}

//------------------------------------------
void alccd5_client::takeExposure(int expTime) {
    float fexpt;
    QElapsedTimer *localTimer;

    if (alccd5->isConnected()) {
        ccd_exposure = alccd5->getNumber("CCD_EXPOSURE");
        if (ccd_exposure == NULL)     {
            qDebug() << "Error: unable to find CCD_EXPOSURE property...";
            return;
        }
        localTimer = new QElapsedTimer();
        localTimer->start();
        fexpt=(float)expTime;
        while (localTimer->elapsed() < 100) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        }
        localTimer->restart();
        ccd_exposure->np[0].value = fexpt;
        while (localTimer->elapsed() < 100) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        }
        delete localTimer;
        if ((fexpt > 0.001) && (fexpt < 3600)) {
            sendNewNumber(ccd_exposure);
        }
    } else {
        qDebug() << "Cam not connected...";
    }
}

//------------------------------------------
void alccd5_client::newDevice(INDI::BaseDevice *dp) {
    if (!strcmp(dp->getDeviceName(), MYCCD)) {
        qDebug() << "Receiving Device:" << dp->getDeviceName();
        this->serverMessage->clear();
        this->serverMessage->append(dp->getDeviceName());
        emit messageFromINDIAvailable();
    }
    alccd5 = dp;
}

//------------------------------------------

QString* alccd5_client::getINDIServerMessage(void) {
    return serverMessage;
}

//------------------------------------------
void alccd5_client::newProperty(INDI::Property *property) {
    if (!strcmp(property->getDeviceName(), MYCCD) && !strcmp(property->getName(), "CONNECTION")) {
        connectDevice(MYCCD);
        return;
    }
}

//------------------------------------------
void alccd5_client::newNumber(INumberVectorProperty *nvp) { }

//------------------------------------------
void alccd5_client::newMessage(INDI::BaseDevice *dp, int messageID) {
     if (strcmp(dp->getDeviceName(), MYCCD)) {
         return;
     }
     qDebug() << "Receiving message from Server: " << dp->messageQueue(messageID).c_str();
     this->serverMessage->clear();
     this->serverMessage->append(dp->messageQueue(messageID).c_str());
     emit messageFromINDIAvailable();
}

//------------------------------------------

bool alccd5_client::getCCDParameters(void) {
    INumberVectorProperty *ccd_params;

    qDebug() << "Retrieving camera data";
    if (alccd5->isConnected()) {
        ccd_params = alccd5->getNumber("CCD_INFO");
        if (ccd_params == NULL)     {
            qDebug() << "Error: unable to find CCD_INFO property...";
            return 0;
        }
            this->pixSizeX = IUFindNumber(ccd_params,"CCD_PIXEL_SIZE_X")->value;
            this->pixSizeY = IUFindNumber(ccd_params,"CCD_PIXEL_SIZE_Y")->value;
            this->frameSizeX = IUFindNumber(ccd_params,"CCD_MAX_X")->value;
            this->frameSizeY = IUFindNumber(ccd_params,"CCD_MAX_Y")->value;
            this->bitsPerPixel = IUFindNumber(ccd_params, "CCD_BITSPERPIXEL")->value;
    } else {
        qDebug() << "Cam not connected...";
        return 0;
    }
    g_AllData->setCameraParameters(this->pixSizeX,this->pixSizeY,this->frameSizeX,this->frameSizeY);
    return 1;
}

//------------------------------------------

void alccd5_client::newBLOB(IBLOB *bp) {
    // receive a FITS file from the server ...
    // header is 2880, image is 1280x1024
    char *fitsdata;
    QImage *smallQImage, *mimage;
    ofstream fitsfile;
    int imgwidth, imgheight,widgetWidth,widgetHeight;
    float sfw,sfh;

    fitsdata = static_cast<char *>(bp->blob);
    // casting the INDI-BLOB containing the image data in FITS to an array of uints ...

    fitsdata=fitsdata+2880;
    // skipping the header which is 2880 bytes for the fits from the QHY5

    imgwidth = g_AllData->getCameraChipPixels(0);
    imgheight = g_AllData->getCameraChipPixels(1);
    //retrieving the number of pixels on the chip

    fitsqimage = new QImage((uchar*)fitsdata, imgwidth, imgheight, QImage::Format_Indexed8);
    fitsqimage->setColorTable(*myVec);
    mimage = new QImage(fitsqimage->mirrored(0,1));
    // read the image data into a QImage, set a grayscale LUT, and mirror the image ...
    //mimage->save("TestCameraImage.jpg",0,-1);
    // uncomment if you want to see the QImage ...

    widgetWidth=g_AllData->getCameraDisplaySize(0);
    widgetHeight=g_AllData->getCameraDisplaySize(1);
    sfw = widgetWidth/(float)imgwidth;
    sfh = widgetHeight/(float)imgheight;
    if (sfw > sfh) {
        g_AllData->setCameraImageScalingFactor(sfw);
    } else {
        g_AllData->setCameraImageScalingFactor(sfh);
    }
    // get the scaling factor for scaling the QImage to the QPixmap

    smallQImage = new QImage(mimage->scaled(widgetWidth,widgetHeight,Qt::KeepAspectRatio,Qt::FastTransformation));
    //smallQImage->save("SmallTestCameraImage.jpg",0,-1);
    // uncomment if you want to see the QImage ...

    displayPMap->convertFromImage(*smallQImage,0);
    delete smallQImage;
    delete mimage;
    emit this->imageAvailable();
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

void alccd5_client::sayGoodbyeToINDIServer(void) {
    this->disconnectServer();
}

//------------------------------------------
