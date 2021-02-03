// this code is part of "TSC", a free control software for astronomical telescopes
// Copyright (C)  2016-18, wolfgang birkfellner
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//---------------------------------------------------
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
#include <fitsio.h>
#include <QMessageBox>


using namespace std;

extern TSC_GlobalData *g_AllData;

//------------------------------------------
ccd_client::ccd_client() {
    QRgb cval;
    int i;

    this->storeCamImages = false;
    this->ccd[0] = NULL;
    this->ccd[1] = NULL;
    this->fitsqimage = this->mainCamImage = nullptr;
    this->scaledfitsdata[0] = scaledfitsdata[1] = nullptr;
    this->displayPMap = new QPixmap();
    this->mainCCDPMap = new QPixmap();
    this->serverMessage= new QString();
    this->ccdINDIName[0] = new QString();
    this->ccdINDIName[1] = new QString();
    this->guiderCCDName =new QString();
    this->propertyName = new QString();
    this->propertyLabel = new QString();
    this->propertyGroup = new QString();
    this->tprop = new QString();
    this->myVec =new QVector<QRgb>(256);
    for(i=0;i<256;i++) {
        cval = qRgb(i,i,i);
        this->myVec->insert(i, cval);
    }
    // setting colortable for grayscale QImages

    for (i=0; i < 500; i++) {
        this->propertyArray[i][0] = nullptr;
        this->propertyArray[i][1] = nullptr;
    }
    this->noOfProperties[0] = noOfProperties[1] = 0;
    this->expcounter=1;
    this->simulatorCounter=10; // a helper for debugging
}

//------------------------------------------
ccd_client::~ccd_client() {

    delete displayPMap;
    delete mainCCDPMap;
    qDebug() << "Deleted PixMaps";
    delete myVec;
    delete serverMessage;
    delete guiderCCDName;
    delete propertyName;
    delete propertyLabel;
    delete propertyGroup;
    delete tprop;
    qDebug() << "CCD client deleted ...";
}

//------------------------------------------
void ccd_client::setIdxOfCCDs(short idx, bool isMainCCD) {
    if ((idx != 0) && (idx != 1)) {
        qDebug() << "Invalid CCD index set...";
        return;
    }
    if (isMainCCD == true) {
        this->idxOfMainCCD = idx;
    } else {
        this->idxOfGuiderCCD = idx;
        this->guiderCCDName = this->ccdINDIName[idx]; // we need to know the name of the guiderdevice in order to assign BLOBS
    }
}

//------------------------------------------
QString* ccd_client::getINDIName(short what) {
    switch (what) {
        case 0: return this->ccdINDIName[0];
        case 1: return this->ccdINDIName[1];
        default: return NULL;
    }
}
//------------------------------------------
bool ccd_client::probeForCCD(short idx) {

    if (ccd[idx] == NULL) {
        qDebug() << "CCD " << idx << "not connected!";
        return false;
    }
    return true;
}

//------------------------------------------
bool ccd_client::setINDIServer(QString addr, int port) {
    bool serverconnected;

    qDebug() << "Setting INDI Server...";
    QByteArray ba = addr.toLatin1(); // convert the server tcp/ip address to a byte array
    const char *c_str2 = ba.data();  // cast the qbytearray data to a string
    this->setServer(c_str2, port); // set the server
    serverconnected= this->connectServer();
    return serverconnected;
}

//-------------------------------------------
void ccd_client::removeDevice(INDI::BaseDevice *dp){
    qDebug() << "Device" << dp->getDeviceName() << "removed.";
}

//-------------------------------------------
void ccd_client::serverDisconnected(int exit_code) {
   qDebug() << "INDI Server disconnected with code " << exit_code;
}
//-------------------------------------------
void ccd_client::newSwitch(ISwitchVectorProperty *svp){
    if (!strcmp(svp->device,this->guiderCCDName->toLatin1())) {
        emit this->switchSetOnServerGuiderCCD();
    } else {
        emit this->switchSetOnServerMainCCD();
    }
    QCoreApplication::processEvents(QEventLoop::AllEvents,100);
}

//------------------------------------------
void ccd_client::newNumber(INumberVectorProperty *nvp) {
    if (!strcmp(nvp->device,this->guiderCCDName->toLatin1())) {
        emit this->numberSetOnServerGuiderCCD();
    } else {
        emit this->numberSetOnServerMainCCD();
    }
    QCoreApplication::processEvents(QEventLoop::AllEvents,100);
}
//------------------------------------------

void ccd_client::newText(ITextVectorProperty *tvp) {
    if (!strcmp(tvp->device,this->guiderCCDName->toLatin1())) {
        emit this->textSetOnServerGuiderCCD();
    } else {
        emit this->textSetOnServerMainCCD();
    }
    QCoreApplication::processEvents(QEventLoop::AllEvents,100);
}
//------------------------------------------
void ccd_client::newLight(ILightVectorProperty *lvp) {
    qDebug() << "New light value arrived set server: " << lvp->name ;
}

//------------------------------------------
void ccd_client::disconnectFromServer(void) {
    int i;

    this->disconnectDevice(ccdINDIName[0]->toLatin1());
    this->disconnectDevice(ccdINDIName[1]->toLatin1());
    this->ccd[0] = NULL;
    this->ccd[1] = NULL;
    this->idxOfGuiderCCD = 0;
    this->idxOfMainCCD = 0;
    this->noOfCams = 0;
    this->guiderCCDName->clear();
    this->firstDeviceConnected = false;
    this->secondDeviceConnected = false;
    this->ccd_exposure = NULL; // an INDI data structure on exposure time
    this->ccd_gain = NULL; // an INDI data structure on camera gain
    this->ccd_params = NULL;
    this->noOfProperties[0] = 0;
    this->noOfProperties[1] = 0;
    for (i = 0; i < this->noOfProperties[0]; i++) {
        this->propertyArray[noOfProperties[0]][i] = NULL;
    }
    for (i = 0; i < this->noOfProperties[1]; i++) {
        this->propertyArray[noOfProperties[1]][i] = NULL;
    }
    this->noOfProperties[0] = 0;
    this->noOfProperties[1] = 0;
    this->disconnectServer();
}

//------------------------------------------
void ccd_client::takeExposure(int expTime, bool isMainCCD) {
    float fexpt;
    QElapsedTimer *localTimer;
    short idx;

    if (isMainCCD == true) {
        idx = this->idxOfMainCCD;
    } else {
        idx = this->idxOfGuiderCCD;
    }

    if (this->probeForCCD(idx) == false) {
        qDebug() << "CCD not found ...";
        return;
    }
    if (ccd[idx]->isConnected()) {
        ccd_exposure = ccd[idx]->getNumber("CCD_EXPOSURE");
        if (ccd_exposure == NULL)     {
            qDebug() << "Exposure not successful ...";
            return;
        }
        qDebug() << "Taking exposure...";
        fexpt=(float)expTime;
        localTimer = new QElapsedTimer(); // INDIserver needs a few ms to digest the command ...
        localTimer->start();
        ccd_exposure->np[0].value = fexpt;
        while (localTimer->elapsed() < 100) {
            QCoreApplication::processEvents(QEventLoop::AllEvents,250);
        }
        delete localTimer;
        if ((fexpt > 0.001) && (fexpt < 3600)) {
            sendNewNumber(ccd_exposure);
        }
    }
}

//------------------------------------------
void ccd_client::newDevice(INDI::BaseDevice *dp) {

    qDebug() << "New device: " << (dp->getDeviceName());
    if (this->noOfCams == 0) {
        this->ccd[0] = dp;
        this->ccdINDIName[0]->clear();
        this->ccdINDIName[0]->append(dp->getDeviceName());
        this->noOfCams = 1;
        this->watchDevice(this->ccdINDIName[0]->toLatin1());
        this->setBLOBMode(B_ALSO, this->ccdINDIName[0]->toLatin1(), NULL);
    } else {
        this->ccd[1] = dp;
        this->ccdINDIName[1]->clear();
        this->ccdINDIName[1]->append(dp->getDeviceName());
        this->noOfCams = 2;
        this->watchDevice(this->ccdINDIName[1]->toLatin1());
        this->setBLOBMode(B_ALSO, this->ccdINDIName[1]->toLatin1(), NULL);
    }
}

//------------------------------------------

QString* ccd_client::getINDIServerMessage(void) {
    return serverMessage;
}

//------------------------------------------
void ccd_client::newProperty(INDI::Property *property) {

    if (!strcmp(property->getDeviceName(), this->ccdINDIName[0]->toLatin1())) {
        this->propertyArray[noOfProperties[0]][0] = property;
        this->noOfProperties[0]++;
        emit newPropertyListArrived();
        QCoreApplication::processEvents(QEventLoop::AllEvents,1000);
        qDebug() << "New Cam 1 property: " << property->getName();

    }
    if (!strcmp(property->getDeviceName(), this->ccdINDIName[1]->toLatin1())) {
        this->propertyArray[noOfProperties[1]][1] = property;
        this->noOfProperties[1]++;
        emit newPropertyListArrived();
        QCoreApplication::processEvents(QEventLoop::AllEvents,1000);
        qDebug() << "New Cam 2 property: " << property->getName();
    }

    if (!strcmp(property->getDeviceName(), this->ccdINDIName[0]->toLatin1()) && !strcmp(property->getName(), "CONNECTION")) {
        connectDevice(this->ccdINDIName[0]->toLatin1());
        this->firstDeviceConnected = true;
        qDebug() << "First Camera connected ...";
    }
    if (!strcmp(property->getDeviceName(), this->ccdINDIName[1]->toLatin1()) && !strcmp(property->getName(), "CONNECTION")) {
        connectDevice(this->ccdINDIName[1]->toLatin1());
        this->secondDeviceConnected = true;
        qDebug() << "Second Camera connected ...";
    }
    return;
}

//---------------------------------------------------------------------------------
void ccd_client::removeProperty(INDI::Property *property) {
    INDI::Property *intermedPropertyArray[500];
    int i, cnt;

    if (!strcmp(property->getDeviceName(), this->ccdINDIName[0]->toLatin1())) {
        for (i = 0, cnt = 0; i < this->noOfProperties[0]; i++) {
            if (strcmp(property->getName(), this->propertyArray[i][0]->getName()) != 0) {
                intermedPropertyArray[cnt]=this->propertyArray[i][0];
                cnt++;
            }
        } // copied n-1 properties
        this->noOfProperties[0] = cnt;
        for (i = 0; i < this->noOfProperties[0]; i++) {
            this->propertyArray[i][0] = intermedPropertyArray[i];
        }
        emit newPropertyListArrived();
        QCoreApplication::processEvents(QEventLoop::AllEvents,1000);
        qDebug() << "Removed property, new number is " << this->noOfProperties[0];
    }
    if (!strcmp(property->getDeviceName(), this->ccdINDIName[1]->toLatin1())) {
        for (i = 0, cnt = 0; i < this->noOfProperties[1]; i++) {
            if (strcmp(property->getName(), this->propertyArray[i][1]->getName()) != 0) {
                intermedPropertyArray[cnt]=this->propertyArray[i][1];
                cnt++;
            }
        } // copied n-1 properties
        this->noOfProperties[1] = cnt;
        for (i = 0; i < this->noOfProperties[1]; i++) {
            this->propertyArray[i][1] = intermedPropertyArray[i];
        }
        emit newPropertyListArrived();
        QCoreApplication::processEvents(QEventLoop::AllEvents,1000);
        qDebug() << "Removed property, new number is " << this->noOfProperties[1];

    }

}

//------------------------------------------
void ccd_client::newMessage(INDI::BaseDevice *dp, int messageID) {
     if (strcmp(dp->getDeviceName(), this->ccdINDIName[0]->toLatin1())) { // this is a workaround for the time being
         return;
     }
     this->serverMessage->clear();
     this->serverMessage->append(dp->messageQueue(messageID).c_str());
     emit messageFromINDIAvailable();
     QCoreApplication::processEvents(QEventLoop::AllEvents,100);
}

//------------------------------------------
bool ccd_client::getCCDParameters(bool isMainCCD) {
    QElapsedTimer *localTimer;
    long imgsize;
    short idx;

    if (isMainCCD == true) {
        idx = this->idxOfMainCCD;
    } else {
        idx = this->idxOfGuiderCCD;
    }
    qDebug() << "---------------------------------------------";
    qDebug() << "Getting camera parameters for Index: " << idx;
    if (ccd[idx]->isConnected()) {
        if (ccd[idx] != NULL) {
            ccd_params = ccd[idx]->getNumber("CCD_INFO");
            if (ccd_params == NULL) {
                qDebug() << "Could not retrieve CCD information ...";
                return false;
            }
        } else {
            qDebug() << "No info for ccd " << idx;
            return false;
        }
        this->pixSizeX = IUFindNumber(ccd_params,"CCD_PIXEL_SIZE_X")->value;
        this->pixSizeY = IUFindNumber(ccd_params,"CCD_PIXEL_SIZE_Y")->value;
        this->frameSizeX = IUFindNumber(ccd_params,"CCD_MAX_X")->value;
        this->frameSizeY = IUFindNumber(ccd_params,"CCD_MAX_Y")->value;
        this->bitsPerPixel = IUFindNumber(ccd_params, "CCD_BITSPERPIXEL")->value;
        qDebug() << pixSizeX << "/" << pixSizeY << "/" << frameSizeX << "/" << frameSizeY << "/" << bitsPerPixel;
        g_AllData->setCameraParameters(this->pixSizeX,this->pixSizeY,this->frameSizeX,this->frameSizeY,isMainCCD);
        g_AllData->setCameraBitDepth((int)this->bitsPerPixel,isMainCCD);
        imgsize=(this->frameSizeX*this->frameSizeY);
        if (scaledfitsdata[idx] == nullptr) {
            scaledfitsdata[idx] = new char [imgsize];
        }
    } else {
        qDebug() << "Camera not connected...";
        return false;
    }
    // now send a very short exposure to get a FITS header for additional information - and for waking the ZWO 120MM up ...
    if (isMainCCD == false) {
        ccd_exposure = ccd[idx]->getNumber("CCD_EXPOSURE");
        if (ccd_exposure == NULL)     {
            qDebug() << "Exposure not successful ...";
            return false;
        }
        localTimer = new QElapsedTimer(); // INDIserver needs a few ms to digest the command ...
        localTimer->start();
        ccd_exposure->np[0].value = 0.01;
        while (localTimer->elapsed() < 100) {
            QCoreApplication::processEvents(QEventLoop::AllEvents,250);
        }
        delete localTimer;
        this->isAProbeImage = true;
        sendNewNumber(ccd_exposure);
    }
    return true;
}

//------------------------------------------
// receive a FITS file from the server.
void ccd_client::newBLOB(IBLOB *bp) {
    char *fitsdata = nullptr;
    uint16_t *_16bitdata = nullptr;
    uint16_t _16BitPixValA,_16BitPixValB, _16BitPixVal;
    uint16_t histogram[66000];
    long indFITS, indRAWFITS,lowerHInt, upperHInt, uCount, hCount, imax, imin;
    QImage *smallQImage;
    int imgwidth, imgheight,widgetWidth,widgetHeight;
    double imgDepth;
    float sfw,sfh, percentageClipped;
    QString *efilename, *ametryFileName;
    bool isMainCCD = false;

    if (this->guiderCCDName->compare(bp->bvp->device) == 0) { // we have an image from the guidecam ...
        qDebug() << "Received Guider Image...";
        isMainCCD = false;
        imgwidth = g_AllData->getCameraChipPixels(0,false);
        imgheight = g_AllData->getCameraChipPixels(1,false);
        imgDepth = g_AllData->getCameraBitDepth(false);
        //retrieving the number of pixels on the chip
    } else {
        isMainCCD = true;
        imgwidth = g_AllData->getCameraChipPixels(0,true);
        imgheight = g_AllData->getCameraChipPixels(1,true);
        imgDepth = g_AllData->getCameraBitDepth(true);
    }

    fitsdata = (static_cast<char *>(bp->blob));
    fitsdata=fitsdata+2880;     // header is 2880
        // skipping the header which is 2880 bytes for the fits from the server

    if (imgDepth == 16) { // handling a 16 bit image
        _16bitdata = new uint16_t [bp->bloblen-2880];
        if (_16bitdata == nullptr) {
            qDebug() << "couldn't allocate memory for 16 bit image";
        }
        indRAWFITS = 0;
        for (indFITS = 0; indFITS < ((imgwidth*imgheight-2880)); indFITS++) {
            _16BitPixValA = *fitsdata;;
            fitsdata++;
            _16BitPixValB = *fitsdata;
            fitsdata++;
            _16BitPixVal = _16BitPixValA*256+_16BitPixValB;
            _16bitdata[indRAWFITS] = _16BitPixVal-32768;
            indRAWFITS++;
        }
        for (indFITS = 0; indFITS < (imgwidth*imgheight); indFITS++) {
            histogram[_16bitdata[indFITS]] +=1;
        }
        percentageClipped = imgwidth*imgheight/50.0; // the number of pixels to be clipped in the histogram
        uCount = hCount = lowerHInt = upperHInt = 0;
        do {
            lowerHInt +=histogram[uCount];
            uCount++;
        } while (lowerHInt < percentageClipped);
        uCount--;
        hCount = 0;
        do {
            upperHInt += histogram[65535-hCount];
            hCount++;
        } while (upperHInt < percentageClipped);
        hCount--;
        imin=uCount;
        imax=65535-hCount;
        for (indFITS = 0; indFITS < (imgwidth*imgheight); indFITS++) {
            if (_16bitdata[indFITS] < imin) {
                _16bitdata[indFITS] = imin;
            }
            if (_16bitdata[indFITS] > imax) {
                _16bitdata[indFITS] = imax;
            }
        } // so this is a stretch to 96% of the histogram content
        for (indFITS = 0; indFITS < (imgwidth*imgheight); indFITS++) {
            if (isMainCCD == true) {
                scaledfitsdata[idxOfMainCCD][indFITS] = ((char)(floor((_16bitdata[indFITS]-imin)/((float)(imax-imin))*250)));
            } else {
                scaledfitsdata[idxOfGuiderCCD][indFITS] = ((char)(floor((_16bitdata[indFITS]-imin)/((float)(imax-imin))*250)));
                delete _16bitdata;
            }
        }
    }

    if (isMainCCD == false) { // now handle a guider image
        // casting the INDI-BLOB containing the image data in FITS to an array of uints ...
        if (imgDepth == 8) {
            fitsqimage = new QImage((uchar*)fitsdata, imgwidth, imgheight, QImage::Format_Indexed8);
        } else {
            fitsqimage = new QImage((uchar*)scaledfitsdata[idxOfGuiderCCD], imgwidth, imgheight, QImage::Format_Indexed8);
        }
        fitsqimage->setColorTable(*myVec);
        g_AllData->storeCameraImage(*fitsqimage);
        QCoreApplication::processEvents(QEventLoop::AllEvents,250);
        // read the image data into a QImage, set a grayscale LUT, and mirror the image ...
        if (this->storeCamImages==true) {
            efilename=new QString("GuideCameraImage");
            efilename->append(QString::number((double)expcounter,1,0));
            efilename->append(".jpg");
            fitsqimage->save(efilename->toLatin1(),0,-1);
            this->expcounter++;
            delete efilename;
        }
        widgetWidth=g_AllData->getCameraDisplaySize(0,false);
        widgetHeight=g_AllData->getCameraDisplaySize(1,false);
        sfw = widgetWidth/(float)imgwidth;
        sfh = widgetHeight/(float)imgheight;
        if (sfw < sfh) {
            g_AllData->setCameraImageScalingFactor(sfw,false);
        } else {
            g_AllData->setCameraImageScalingFactor(sfh,false);
        }
        // get the scaling factor for scaling the QImage to the QPixmap

        //-----------------------------------------------------------------
         // guide debugging code --- load a camera image for debugging here ...
      /*   efilename=new QString("GuideSimulatorImages/TestCameraImage");
         efilename->append(QString::number((double)simulatorCounter,1,0));
         efilename->append(".jpg");
         this->simulatorCounter++;
         if (this->simulatorCounter > 40) {
             this->simulatorCounter=10;
         }
         fitsqimage = new QImage(efilename->toLatin1());
         delete efilename;
         g_AllData->storeCameraImage(*fitsqimage);

    */
         //-----------------------------------------------------------------

        smallQImage = new QImage(fitsqimage->scaled(widgetWidth,widgetHeight,Qt::KeepAspectRatio,Qt::FastTransformation));
        this->displayPMap->convertFromImage(*smallQImage,0);
        delete smallQImage;
        emit this->imageAvailable(displayPMap);
        QCoreApplication::processEvents(QEventLoop::AllEvents,250);
    } else {
        qDebug() << "Got Main Camera Image ...";

        // casting the INDI-BLOB containing the image data in FITS to an array of uints ...
        if (imgDepth == 8) {
            mainCamImage = new QImage((uchar*)fitsdata, imgwidth, imgheight, QImage::Format_Indexed8);
        } else {
            mainCamImage = new QImage((uchar*)scaledfitsdata[idxOfMainCCD], imgwidth, imgheight, QImage::Format_Indexed8);
        }
        mainCamImage->setColorTable(*myVec);
        ametryFileName = new QString(g_AllData->getPathToImages());
        ametryFileName->append("solve.jpg");
        mainCamImage->save(ametryFileName->toLatin1(),0,100);
        g_AllData->setPathToImageToBeSolved(ametryFileName->toLatin1());
        delete ametryFileName;
        widgetWidth=g_AllData->getCameraDisplaySize(0,true);
        widgetHeight=g_AllData->getCameraDisplaySize(1,true);
        sfw = widgetWidth/(float)imgwidth;
        sfh = widgetHeight/(float)imgheight;
        if (sfw < sfh) {
            g_AllData->setCameraImageScalingFactor(sfw,true);
        } else {
            g_AllData->setCameraImageScalingFactor(sfh,true);
        }
        smallQImage = new QImage(mainCamImage->scaled(widgetWidth,widgetHeight,Qt::KeepAspectRatio,Qt::FastTransformation));
        this->mainCCDPMap->convertFromImage(*smallQImage,0);
        delete smallQImage;
        delete mainCamImage;
        emit this->mainCCDImageAvailable(mainCCDPMap);
        QCoreApplication::processEvents(QEventLoop::AllEvents,1000);
        qDebug() << "Main image processed and signal emitted ...";
        // handle main camera image
    }
}

//------------------------------------------

void ccd_client::sayGoodbyeToINDIServer(void) {
    this->disconnectFromServer();
}

//------------------------------------------

void ccd_client::setStoreImageFlag(bool what) {
    this->storeCamImages=what;
}

//------------------------------------------
// routines for getting and setting values from INDIserver

QString* ccd_client::getPropertyName(bool isMainCCD, int idx) {
    if (isMainCCD == true) {
        if (idx < this->noOfProperties[this->idxOfMainCCD]) {
            this->propertyName->clear();
            this->propertyName->append(this->propertyArray[idx][this->idxOfMainCCD]->getName());
            return propertyName;
        } else {
            return NULL;
        }
    } else {
        if (idx < this->noOfProperties[this->idxOfGuiderCCD]) {
            this->propertyName->clear();
            this->propertyName->append(this->propertyArray[idx][this->idxOfGuiderCCD]->getName());
            return propertyName;
        } else  {
            return NULL;
        }
    }
}
//------------------------------------------

QString* ccd_client::getPropertyGroup(bool isMainCCD, int idx) {
    if (isMainCCD == true) {
        if (idx < this->noOfProperties[this->idxOfMainCCD]) {
            this->propertyGroup->clear();
            this->propertyGroup->append(this->propertyArray[idx][this->idxOfMainCCD]->getGroupName());
            return propertyGroup;
        } else {
            return NULL;
        }
    } else {
        if (idx < this->noOfProperties[this->idxOfGuiderCCD]) {
            this->propertyGroup->clear();
            this->propertyGroup->append(this->propertyArray[idx][this->idxOfGuiderCCD]->getGroupName());
            return propertyGroup;
        } else  {
            return NULL;
        }
    }
}
//------------------------------------------
QString* ccd_client::getPropertyLabel(bool isMainCCD, int idx) {
    if (isMainCCD == true) {
        if (idx < this->noOfProperties[this->idxOfMainCCD]) {
            this->propertyLabel->clear();
            this->propertyLabel->append(this->propertyArray[idx][this->idxOfMainCCD]->getLabel());
            return propertyLabel;
        } else {
            return NULL;
        }
    } else {
        if (idx < this->noOfProperties[this->idxOfGuiderCCD]) {
            this->propertyLabel->clear();
            this->propertyLabel->append(this->propertyArray[idx][this->idxOfGuiderCCD]->getLabel());
            return propertyLabel;
        } else {
            return NULL;
        }
    }
}
//------------------------------------------
INDI_PROPERTY_TYPE ccd_client::getPropertyType(bool isMainCCD, int idx) {
    if (isMainCCD == true) {
        if (idx <= this->noOfProperties[this->idxOfMainCCD]) {
            return this->propertyArray[idx][this->idxOfMainCCD]->getType();
        }
    } else {
        if (idx <= this->noOfProperties[this->idxOfGuiderCCD]) {
            return this->propertyArray[idx][this->idxOfGuiderCCD]->getType();
        }
    }
    return INDI_UNKNOWN;
}

//------------------------------------------
int ccd_client::getNoOfValuesInProperty(bool isMainCCD, int idx) {
    int retval = -1, camIdx;
    INDI_PROPERTY_TYPE currType;

    if (isMainCCD == true) {
        camIdx = this->idxOfMainCCD;
    } else {
        camIdx = this->idxOfGuiderCCD;
    }
    if (idx < this->noOfProperties[camIdx]) {
        currType = this->propertyArray[idx][camIdx]->getType();
        switch (currType) {
        case INDI_NUMBER:
            retval = this->propertyArray[idx][camIdx]->getNumber()->nnp;
            break;
        case INDI_TEXT:
            retval = this->propertyArray[idx][camIdx]->getText()->ntp;
            break;
        case INDI_SWITCH:
            retval = this->propertyArray[idx][camIdx]->getSwitch()->nsp;
            break;
        case INDI_LIGHT:
            retval = this->propertyArray[idx][camIdx]->getLight()->nlp;
            break;
        case INDI_BLOB:
            retval = this->propertyArray[idx][camIdx]->getBLOB()->nbp;
            break;
        case INDI_UNKNOWN:
            retval = -1;
            break;
        default:
            retval = -1;
            break;
        }
    }
    return retval;
}
//------------------------------------------
int ccd_client::getNoOfProperties(bool isMainCCD) {
    if (isMainCCD == true) {
        return noOfProperties[this->idxOfMainCCD];
    } else {
        return noOfProperties[this->idxOfGuiderCCD];
    }
}

//------------------------------------------

QString* ccd_client::getSwitchPropertyItemLabel(bool isMainCCD, int idx, int cnt) {
    int camIdx;

    if (isMainCCD == true) {
        camIdx = this->idxOfMainCCD;
    } else {
        camIdx = this->idxOfGuiderCCD;
    }

    if (idx < this->noOfProperties[camIdx]) {
        this->currentSwitchVector = this->propertyArray[idx][camIdx]->getSwitch();
        if (cnt < this->currentSwitchVector->nsp) {
            this->tprop->clear();
            this->tprop->append((this->currentSwitchVector->sp[cnt].label));
            return tprop;
        }
    }
    return NULL;
}

//------------------------------------------
QString* ccd_client::getNumberPropertyItemLabel(bool isMainCCD, int idx, int cnt) {
    int camIdx;

    if (isMainCCD == true) {
        camIdx = this->idxOfMainCCD;
    } else {
        camIdx = this->idxOfGuiderCCD;
    }

    if (idx < this->noOfProperties[camIdx]) {
        this->currentNumberVector = this->propertyArray[idx][camIdx]->getNumber();
        if (cnt < this->currentNumberVector->nnp) {
            this->tprop->clear();
            this->tprop->append((this->currentNumberVector->np[cnt].label));
            return tprop;
        }
    }
    return NULL;
}

//------------------------------------------
QString* ccd_client::getTextPropertyItemLabel(bool isMainCCD, int idx, int cnt) {
    int camIdx;

    if (isMainCCD == true) {
        camIdx = this->idxOfMainCCD;
    } else {
        camIdx = this->idxOfGuiderCCD;
    }

    if (idx < this->noOfProperties[camIdx]) {
        this->currentTextVector = this->propertyArray[idx][camIdx]->getText();
        if (cnt < this->currentTextVector->ntp) {
            this->tprop->clear();
            this->tprop->append((this->currentTextVector->tp[cnt].label));
            return tprop;
        }
    }

    return NULL;
}

//------------------------------------------
QString* ccd_client::getLightPropertyItemLabel(bool isMainCCD, int idx, int cnt) {
    int camIdx;

    if (isMainCCD == true) {
        camIdx = this->idxOfMainCCD;
    } else {
        camIdx = this->idxOfGuiderCCD;
    }

    if (idx < this->noOfProperties[camIdx]) {
        this->currentLightVector = this->propertyArray[idx][camIdx]->getLight();
        if (cnt < this->currentLightVector->nlp) {
            this->tprop->clear();
            this->tprop->append((this->currentLightVector->lp[cnt].label));
            return tprop;
        }
    }
    return NULL;
}

//------------------------------------------
QString* ccd_client::getBLOBPropertyItemLabel(bool isMainCCD, int idx, int cnt) {
    int camIdx;

    if (isMainCCD == true) {
        camIdx = this->idxOfMainCCD;
    } else {
        camIdx = this->idxOfGuiderCCD;
    }
    if (idx < this->noOfProperties[camIdx]) {
        this->currentBLOBVector = this->propertyArray[idx][camIdx]->getBLOB();
        if (cnt < this->currentBLOBVector->nbp) {
            this->tprop->clear();
            this->tprop->append((this->currentBLOBVector->bp[cnt].label));
            return tprop;
        }
    }
    return NULL;
}

//------------------------------------------
// a function that return read/write permissions for properties
bool ccd_client::getRWPermission(bool isMainCCD, int idx, bool isReadPerm) {
    int camIdx;
    bool retval = false;
    IPerm permission = IP_RO;
    INDI_PROPERTY_TYPE currType;

    if (isMainCCD == true) {
        camIdx = this->idxOfMainCCD;
    } else {
        camIdx = this->idxOfGuiderCCD;
    }
    if (idx < this->noOfProperties[camIdx]) {
        currType = this->propertyArray[idx][camIdx]->getType();
        switch (currType) {
        case INDI_NUMBER:
            permission = this->propertyArray[idx][camIdx]->getNumber()->p;
            break;
        case INDI_TEXT:
            permission = this->propertyArray[idx][camIdx]->getText()->p;
            break;
        case INDI_SWITCH:
            permission = this->propertyArray[idx][camIdx]->getSwitch()->p;
            break;
        case INDI_LIGHT:
            permission = IP_RO; // lights cannot be set
            break;
        case INDI_BLOB:
            permission = this->propertyArray[idx][camIdx]->getBLOB()->p;
            break;
        case INDI_UNKNOWN:
            retval = false;
            break;
        default:
            retval = false;
            break;
        }
    }
    retval = false;
    if (isReadPerm == true) {
        if ((permission == IP_RO) || (permission == IP_RW)) {
            retval = true;
        }
    } else {
        if ((permission == IP_WO) || (permission == IP_RW)) {
            retval = true;
        }
    }
    return retval;
}

//----------------------------------------------------------------------
INumber ccd_client::getCurrentNumber(bool isMainCCD, int idx, int cnt) {
    int camIdx;

    this->nprop.max = 0;
    this->nprop.min = 0;
    this->nprop.step = 0;
    this->nprop.value = 0;
    if (isMainCCD == true) {
        camIdx = this->idxOfMainCCD;
    } else {
        camIdx = this->idxOfGuiderCCD;
    }
    if (idx < this->noOfProperties[camIdx]) {
        this->currentNumberVector = this->propertyArray[idx][camIdx]->getNumber();
        if (cnt < this->currentNumberVector->nnp) {
            this->nprop = (this->currentNumberVector->np[cnt]);
            return nprop;
        }
    }
    return nprop;
};

//---------------------------------------------------------------------
void ccd_client::setCurrentNumberValue(bool isMainCCD, int idx, int cnt, double val) {
    int camIdx;
    double max, min;


    if (isMainCCD == true) {
        camIdx = this->idxOfMainCCD;
    } else {
        camIdx = this->idxOfGuiderCCD;
    }
    if (idx < this->noOfProperties[camIdx]) {
        this->currentNumberVector = this->propertyArray[idx][camIdx]->getNumber();
        if ((cnt < this->currentNumberVector->nnp) && (cnt >= 0)) {
            this->nprop = (this->currentNumberVector->np[cnt]);
            max = nprop.max;
            min = nprop.min;
            if ((val <= max) && (val >= min)) {
                this->currentNumberVector->np[cnt].value = val;
            }
        }
    }
}

//---------------------------------------------------------------------
// this one is invoked when all numbers in the currentVector were set
void ccd_client::sendCurrentNumberValueCompound(void) {
    QElapsedTimer* localTimer;


    if (this->currentNumberVector != NULL) {
        sendNewNumber(this->currentNumberVector);
        localTimer = new QElapsedTimer(); // INDIserver needs a few ms to digest the command ...
        localTimer->start();
        while (localTimer->elapsed() < 100) {
            QCoreApplication::processEvents(QEventLoop::AllEvents,250);
        }
        delete localTimer;
    }
}

//---------------------------------------------------------------------
QString* ccd_client::getCurrentText(bool isMainCCD, int idx, int cnt) {
    int camIdx;

    if (isMainCCD == true) {
        camIdx = this->idxOfMainCCD;
    } else {
        camIdx = this->idxOfGuiderCCD;
    }

    if (idx < this->noOfProperties[camIdx]) {
        this->currentTextVector = this->propertyArray[idx][camIdx]->getText();
        if ((cnt < this->currentTextVector->ntp) && (cnt >= 0)) {
            this->tprop->clear();
            this->tprop->append(this->currentTextVector->tp[cnt].text);
            return tprop;
        }
    }
    return NULL;
}

//----------------------------------------------------------------------
void ccd_client::setCurrentTextValue(bool isMainCCD, int idx, int cnt, QString textStr) {
    int camIdx;
    QElapsedTimer* localTimer;

    if (isMainCCD == true) {
        camIdx = this->idxOfMainCCD;
    } else {
        camIdx = this->idxOfGuiderCCD;
    }
    if (idx < this->noOfProperties[camIdx]) {
        this->currentTextVector = this->propertyArray[idx][camIdx]->getText();
        if ((cnt < this->currentTextVector->ntp) && (cnt >= 0)) {
            localTimer = new QElapsedTimer(); // INDIserver needs a few ms to digest the command ...
            localTimer->start();
            strcpy(this->currentTextVector->tp[cnt].text, textStr.toLatin1());
            sendNewText(this->currentTextVector);
            while (localTimer->elapsed() < 100) {
                QCoreApplication::processEvents(QEventLoop::AllEvents,250);
            }
            delete localTimer;
        }
    }
}

//------------------------------------------------------------------------
ISRule ccd_client::getSwitchRules(bool isMainCCD, int idx) {
    int camIdx;

    if (isMainCCD == true) {
        camIdx = this->idxOfMainCCD;
    } else {
        camIdx = this->idxOfGuiderCCD;
    }
    if (idx < this->noOfProperties[camIdx]) {
        this->currentSwitchVector = this->propertyArray[idx][camIdx]->getSwitch();
        return this->currentSwitchVector->r;
    }
    qDebug() << "Could not retrieve switch rules ...";
    return ISR_1OFMANY;
}

//-------------------------------------------------------------------------

ISState ccd_client::getCurrentSwitch(bool isMainCCD, int idx, int cnt){
    int camIdx;

    if (isMainCCD == true) {
        camIdx = this->idxOfMainCCD;
    } else {
        camIdx = this->idxOfGuiderCCD;
    }
    if (idx < this->noOfProperties[camIdx]) {
        this->currentSwitchVector = this->propertyArray[idx][camIdx]->getSwitch();
        if (this->currentSwitchVector != NULL) {
            if ((cnt < this->currentSwitchVector->nsp) && (cnt >= 0)) {
                return this->currentSwitchVector->sp[cnt].s;
            }
        } else {
            qDebug() << "Could not retrieve Switch Vector";
        }
    }
    qDebug() << "Could not retrieve value..." << cnt << this->currentSwitchVector->nsp;
    return ISS_OFF;
}

//------------------------------------------------------------------------
void ccd_client::setCurrentSwitch(bool isMainCCD, int idx, int cnt, bool val) {
    int camIdx,i,ci,indArray[64];
    QList<int> cleanIdx;
    QElapsedTimer* localTimer;
    ISState status;
    ISRule rule;

    if (isMainCCD == true) {
        camIdx = this->idxOfMainCCD;
    } else {
        camIdx = this->idxOfGuiderCCD;
    }
    if (idx < this->noOfProperties[camIdx]) {
        this->currentSwitchVector = this->propertyArray[idx][camIdx]->getSwitch();
        if ((cnt < this->currentSwitchVector->nsp) && (cnt >= 0)) {
            if (val == true) {
                status = ISS_ON;
            } else {
                status = ISS_OFF;
            }
            rule = this->currentSwitchVector->r;
            switch (rule) {
            case ISR_1OFMANY:
                for (i = 0; i < this->currentSwitchVector->nsp; i++) {
                    this->currentSwitchVector->sp[i].s = ISS_OFF;
                }
                if (status == ISS_ON) {
                    this->currentSwitchVector->sp[cnt].s = ISS_ON;
                } else {
                    for (i= 0; i <= 63; i++) {
                        indArray[i] = INT_MAX;
                    }
                    for (i= 0; i <= (this->currentSwitchVector->nsp-1); i++) {
                        indArray[i] = i;
                    }
                    indArray[cnt]=INT_MAX;
                    for (i= 0; i <= (this->currentSwitchVector->nsp-1); i++) {
                        cleanIdx << indArray[i];
                    }
                    qStableSort(cleanIdx .begin(), cleanIdx .end());
                    ci = cleanIdx[0];
                    this->currentSwitchVector->sp[cnt].s = ISS_OFF;
                    this->currentSwitchVector->sp[ci].s = ISS_ON; // set the first other switch to ON
                }
                break;
            case ISR_ATMOST1:
                for (i = 0; i < this->currentSwitchVector->nsp; i++) {
                    this->currentSwitchVector->sp[i].s = ISS_OFF;
                }
                this->currentSwitchVector->sp[cnt].s = status; // all can be off, but one can on
                break;
            case ISR_NOFMANY:
                this->currentSwitchVector->sp[cnt].s = status; // all can be on or off
                break;
            }
            localTimer = new QElapsedTimer(); // INDIserver needs a few ms to digest the command ...
            localTimer->start();
            sendNewSwitch(this->currentSwitchVector);
            while (localTimer->elapsed() < 100) {
                QCoreApplication::processEvents(QEventLoop::AllEvents,250);
            }
            delete localTimer;
        }
    }
}

//-----------------------------------------------------------------------------------
// a function that loads an INDI configuration file
void ccd_client::loadINDIConfigFile(bool isMainCCD) {
    int camIdx, idx,cnt;
    QString* propertyName;

    if (isMainCCD == true) {
        camIdx = this->idxOfMainCCD;
    } else {
        camIdx = this->idxOfGuiderCCD;
    }
    propertyName = new QString();
    for (idx = 0; idx < this->noOfProperties[camIdx]; idx++) {
        propertyName->clear();
        propertyName->append(this->getPropertyName(isMainCCD, idx));
        if (propertyName->compare("CONFIG_PROCESS") == 0) {
            this->currentSwitchVector = this->propertyArray[idx][camIdx]->getSwitch();
            for (cnt = 0; cnt < this->currentSwitchVector->nsp; cnt++) {
                if (!strcmp(this->currentSwitchVector->sp[cnt].name,"CONFIG_LOAD")) {
                    this->currentSwitchVector->sp[cnt].s = ISS_ON;
                }
            }
        }
    }
}


//-----------------------------------------------------------------------------------
// a function that stores an INDI configuration file
void ccd_client::saveINDIConfigFile(bool isMainCCD) {
    int camIdx, idx, cnt;
    QString* propertyName;
    bool canSave = false;

    if (isMainCCD == true) {
        camIdx = this->idxOfMainCCD;
    } else {
        camIdx = this->idxOfGuiderCCD;
    }
    propertyName = new QString();
    for (idx = 0; idx < this->noOfProperties[camIdx]; idx++) {
        propertyName->clear();
        propertyName->append(this->getPropertyName(isMainCCD, idx));
        if (propertyName->compare("CONFIG_PROCESS") == 0) {
            this->currentSwitchVector = this->propertyArray[idx][camIdx]->getSwitch();
            for (cnt = 0; cnt < this->currentSwitchVector->nsp; cnt++) {
                if (!strcmp(this->currentSwitchVector->sp[cnt].name,"CONFIG_SAVE")) {
                    this->currentSwitchVector->sp[cnt].s = ISS_ON;
                    canSave = true;
                }
            }
        }
    }
    if (canSave == false) {
        emit saveFunctionNotAvailable();
        QCoreApplication::processEvents(QEventLoop::AllEvents,100);
    }
}
