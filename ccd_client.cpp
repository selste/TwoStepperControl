// derived from INDI client example ...

#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <qdebug.h>
#include <qcolor.h>
#include <qpixmap.h>
#include <baseclient.h>
#include <basedevice.h>
#include <indiproperty.h>
#include "indicom.h"
#include "tsc_globaldata.h"
#include "ccd_client.h"

using namespace std;

extern TSC_GlobalData *g_AllData;

//------------------------------------------
ccd_client::ccd_client() {
    QRgb cval;

    this->storeCamImages = false;
    this->ccd = NULL;
    this->fitsqimage = NULL;
    this->displayPMap = new QPixmap();
    this->serverMessage= new QString();
    this->ccdINDIName = new QString("QHY CCD QHY5-0-M-");
    this->myVec =new QVector<QRgb>(256);
    for(int i=0;i<256;i++) {
        cval = qRgb(i,i,i);
        this->myVec->insert(i, cval);
    }
    // setting colortable for grayscale QImages
    this->expcounter=1;
    this->simulatorCounter=10; // a helper for debugging
}

//------------------------------------------
ccd_client::~ccd_client() {
    delete fitsqimage;
    delete displayPMap;
    delete myVec;
    delete serverMessage;
    delete ccdINDIName;
}

//------------------------------------------
void ccd_client::setCameraName(QString camName) {
    this->ccdINDIName->clear();
    this->ccdINDIName->append(camName.toLatin1());
}

//------------------------------------------
bool ccd_client::setINDIServer(QString addr, int port) {
    bool serverconnected;
    QByteArray ba = addr.toLatin1(); // convert the server tcp/ip address to a byte array
    const char *c_str2 = ba.data();  // cast the qbytearray data to a string

    this->setServer(c_str2, port); // set the server
    serverconnected= this->connectServer();
    return serverconnected;
}

//------------------------------------------
void ccd_client::disconnectFromServer(void) {
    this->disconnectServer();
}

//------------------------------------------
void ccd_client::takeExposure(int expTime) {
    float fexpt;
    QElapsedTimer *localTimer;

    if (ccd->isConnected()) {
        ccd_exposure = ccd->getNumber("CCD_EXPOSURE");
        if (ccd_exposure == NULL)     {
            return;
        }
        localTimer = new QElapsedTimer(); // INDIserver needs a few ms to digest the command ...
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
    }
}

//------------------------------------------
void ccd_client::sendGain(int gain) {
    float fgain;
    QElapsedTimer *localTimer;

    if (ccd->isConnected()) {
        fgain=(float)gain;
        ccd_gain = ccd->getNumber("CCD_GAIN");
        if (ccd_gain==NULL) {
            return;
        }
        localTimer = new QElapsedTimer(); // INDIserver needs a few ms to digest the command ...
        localTimer->start();
        fgain=(float)gain;
        while (localTimer->elapsed() < 100) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        }
        localTimer->restart();
        ccd_gain->np[0].value = fgain;
        while (localTimer->elapsed() < 100) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        }
        delete localTimer;
        sendNewNumber(ccd_gain);
    }
}
//------------------------------------------
void ccd_client::newDevice(INDI::BaseDevice *dp) {

    this->ccd = dp;
    this->ccdINDIName->clear();
    this->ccdINDIName->append(dp->getDeviceName());
    this->watchDevice(this->ccdINDIName->toLatin1());
    this->setBLOBMode(B_ALSO, this->ccdINDIName->toLatin1(), NULL);
}

//------------------------------------------

QString* ccd_client::getINDIServerMessage(void) {
    return serverMessage;
}

//------------------------------------------
void ccd_client::newProperty(INDI::Property *property) {
    if (!strcmp(property->getDeviceName(), this->ccdINDIName->toLatin1()) && !strcmp(property->getName(), "CONNECTION")) {
        connectDevice(this->ccdINDIName->toLatin1());
        return;
    }
}

//------------------------------------------
void ccd_client::newMessage(INDI::BaseDevice *dp, int messageID) {
     if (strcmp(dp->getDeviceName(), this->ccdINDIName->toLatin1())) {
         return;
     }
     this->serverMessage->clear();
     this->serverMessage->append(dp->messageQueue(messageID).c_str());
     emit messageFromINDIAvailable();
}

//------------------------------------------

bool ccd_client::getCCDParameters(void) {
    INumberVectorProperty *ccd_params;

    if (ccd->isConnected()) {
        ccd_params = ccd->getNumber("CCD_INFO");
        if (ccd_params == NULL)     {
            return 0;
        }
            this->pixSizeX = IUFindNumber(ccd_params,"CCD_PIXEL_SIZE_X")->value;
            this->pixSizeY = IUFindNumber(ccd_params,"CCD_PIXEL_SIZE_Y")->value;
            this->frameSizeX = IUFindNumber(ccd_params,"CCD_MAX_X")->value;
            this->frameSizeY = IUFindNumber(ccd_params,"CCD_MAX_Y")->value;
            this->bitsPerPixel = IUFindNumber(ccd_params, "CCD_BITSPERPIXEL")->value;
    } else {
        return 0;
    }
    g_AllData->setCameraParameters(this->pixSizeX,this->pixSizeY,this->frameSizeX,this->frameSizeY);
    return 1;
}

//------------------------------------------
// receive a FITS file from the server.
    // header is 2880, image is 1280x1024
void ccd_client::newBLOB(IBLOB *bp) {
    char *fitsdata;
    QImage *smallQImage;
    int imgwidth, imgheight,widgetWidth,widgetHeight;
    float sfw,sfh;
    QString *efilename;

    fitsdata = static_cast<char *>(bp->blob);
    // casting the INDI-BLOB containing the image data in FITS to an array of uints ...
    fitsdata=fitsdata+2880;
    // skipping the header which is 2880 bytes for the fits from the server
    imgwidth = g_AllData->getCameraChipPixels(0);
    imgheight = g_AllData->getCameraChipPixels(1);
    //retrieving the number of pixels on the chip
    fitsqimage = new QImage((uchar*)fitsdata, imgwidth, imgheight, QImage::Format_Indexed8);
    fitsqimage->setColorTable(*myVec);
    g_AllData->storeCameraImage(*fitsqimage);
    // read the image data into a QImage, set a grayscale LUT, and mirror the image ...
    if (this->storeCamImages==true) {
        efilename=new QString("GuideCameraImage");
        efilename->append(QString::number((double)expcounter,1,0));
        efilename->append(".jpg");
        fitsqimage->save(efilename->toLatin1(),0,-1);
        this->expcounter++;
        delete efilename;
    }
    widgetWidth=g_AllData->getCameraDisplaySize(0);
    widgetHeight=g_AllData->getCameraDisplaySize(1);
    sfw = widgetWidth/(float)imgwidth;
    sfh = widgetHeight/(float)imgheight;
    if (sfw < sfh) {
        g_AllData->setCameraImageScalingFactor(sfw);
    } else {
        g_AllData->setCameraImageScalingFactor(sfh);
    }
    // get the scaling factor for scaling the QImage to the QPixmap

    //-----------------------------------------------------------------
     //-----------------------------------------------------------------
     //-----------------------------------------------------------------
     // guide debugging code --- load a camera image for debugging here ...
 /*    efilename=new QString("GuideSimulatorImages/TestCameraImage");
     efilename->append(QString::number((double)simulatorCounter,1,0));
     efilename->append(".jpg");
     this->simulatorCounter++;
     if (this->simulatorCounter > 40) {
         this->simulatorCounter=10;
     }
     delete mimage;
     mimage = new QImage(efilename->toLatin1());
     delete efilename; */
     //-----------------------------------------------------------------
     //-----------------------------------------------------------------
     //-----------------------------------------------------------------

    g_AllData->storeCameraImage(*fitsqimage);
    smallQImage = new QImage(fitsqimage->scaled(widgetWidth,widgetHeight,Qt::KeepAspectRatio,Qt::FastTransformation));
    displayPMap->convertFromImage(*smallQImage,0);
    delete smallQImage;
    emit this->imageAvailable();
}

//------------------------------------------

QPixmap* ccd_client::getScaledPixmapFromCamera(void) {
    // deliver a small image from the camera to other routines for display
    return displayPMap;
}

//------------------------------------------

void ccd_client::sayGoodbyeToINDIServer(void) {
    this->disconnectServer();
}

//------------------------------------------

void ccd_client::setStoreImageFlag(bool what) {
    this->storeCamImages=what;
}

//------------------------------------------
