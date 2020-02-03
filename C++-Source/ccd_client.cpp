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

using namespace std;

extern TSC_GlobalData *g_AllData;

//------------------------------------------
ccd_client::ccd_client() {
    QRgb cval;

    this->storeCamImages = false;
    this->cameraHasGain = true;
    this->ccd = NULL;
    this->fitsqimage = NULL;
    this->scaledfitsdata = nullptr;
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
bool ccd_client::probeForCCD(void) {
    if (ccd==NULL) {
        qDebug() << "NO CCD!";
        return false;
    } else {
        return true;
    }
}

//------------------------------------------
bool ccd_client::cameraGainAvailable(void) {
    return this->cameraHasGain;
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
    this->ccd = NULL;
    this->disconnectServer();
    this->cameraHasGain = true;
}

//------------------------------------------
void ccd_client::takeExposure(int expTime) {
    float fexpt;
    QElapsedTimer *localTimer;

    if (this->probeForCCD() == false) {
        qDebug() << "CCD not found ...";
        return;
    }
    if (ccd->isConnected()) {
        ccd_exposure = ccd->getNumber("CCD_EXPOSURE");
        if (ccd_exposure == NULL)     {
            qDebug() << "Exposure not successful ...";
            return;
        }
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
void ccd_client::sendGain(int gain) {
    float fgain;
    QElapsedTimer *localTimer;

    if (this->cameraHasGain == true) {
        if (this->probeForCCD() == false) {
            return;
        }
        if (ccd->isConnected()) {
            fgain=(float)gain;
            ccd_gain = ccd->getNumber("CCD_GAIN");
            if (ccd_gain==NULL) {
                qDebug() << "Gain is not available...";
                this->cameraHasGain=false;
                return;
            }
            localTimer = new QElapsedTimer(); // INDIserver needs a few ms to digest the command ...
            localTimer->start();
            fgain=(float)gain;
            while (localTimer->elapsed() < 100) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
            }
            localTimer->restart();
            ccd_gain->np[0].value = fgain;
            while (localTimer->elapsed() < 100) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
            }
            delete localTimer;
            sendNewNumber(ccd_gain);
        }
    }
}
//------------------------------------------
void ccd_client::newDevice(INDI::BaseDevice *dp) {

    qDebug() << "New device: " << (dp->getDeviceName());
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
bool ccd_client::getCCDParameters(bool isMainCCD) {
    INumberVectorProperty *ccd_params;
    QElapsedTimer *localTimer;
    long imgsize;

    if (this->probeForCCD() == false) {
        qDebug() << "CCD not available...";
        return false;
    }
    if (ccd->isConnected()) {
        ccd_params = ccd->getNumber("CCD_INFO");
        if (ccd_params == NULL) {
            qDebug() << "Could not retrieve CCD information ...";
            return false;
        }
        this->pixSizeX = IUFindNumber(ccd_params,"CCD_PIXEL_SIZE_X")->value;
        this->pixSizeY = IUFindNumber(ccd_params,"CCD_PIXEL_SIZE_Y")->value;
        this->frameSizeX = IUFindNumber(ccd_params,"CCD_MAX_X")->value;
        this->frameSizeY = IUFindNumber(ccd_params,"CCD_MAX_Y")->value;
        this->bitsPerPixel = IUFindNumber(ccd_params, "CCD_BITSPERPIXEL")->value;
    } else {
        qDebug() << "Camera not connected...";
        return false;
    }

    g_AllData->setCameraParameters(this->pixSizeX,this->pixSizeY,this->frameSizeX,this->frameSizeY,isMainCCD);
    g_AllData->setCameraBitDepth((int)this->bitsPerPixel,isMainCCD);
    qDebug() << "Information retrieved: " << this->pixSizeX << "/" <<
        this->pixSizeY << "/" << this->frameSizeX << "/" << this->frameSizeY << "/" << this->bitsPerPixel;
    imgsize=(this->frameSizeX*this->frameSizeY);
    if (scaledfitsdata == nullptr) {
        scaledfitsdata = new char [imgsize];
    }
    if (isMainCCD == false) { // this stuff is only done for teh guide camera
        // now probe for gain
        this->sendGain(100);
        // now send a very short exposure to get a FITS header for additional information - and for waking the ZWO 120MM up ...
        ccd_exposure = ccd->getNumber("CCD_EXPOSURE");
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
    // header is 2880, image is 1280x1024
void ccd_client::newBLOB(IBLOB *bp) {
    char *fitsdata = nullptr;
    uint16_t *_16bitdata = nullptr;
    uint16_t _16BitPixValA,_16BitPixValB, _16BitPixVal;
    uint16_t histogram[66000];
    long indFITS, indRAWFITS,lowerHInt, upperHInt, uCount, hCount, imax, imin;
    QImage *smallQImage;
    int imgwidth, imgheight,widgetWidth,widgetHeight;
    float sfw,sfh, percentageClipped;
    QString *efilename;

    if (this->isAProbeImage == true) {
 //       this->saveBLOB(bp);
        this->isAProbeImage = false;
        return;
    }

    imgwidth = g_AllData->getCameraChipPixels(0,false);
    imgheight = g_AllData->getCameraChipPixels(1,false);
    //retrieving the number of pixels on the chip

    fitsdata = (static_cast<char *>(bp->blob));
    fitsdata=fitsdata+2880;
    // skipping the header which is 2880 bytes for the fits from the server

    if (this->bitsPerPixel == 16) {
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
            scaledfitsdata[indFITS] = ((char)(floor((_16bitdata[indFITS]-imin)/((float)(imax-imin))*250)));
        }
        delete _16bitdata;
    }
    // casting the INDI-BLOB containing the image data in FITS to an array of uints ...
    if (this->bitsPerPixel == 8) {
        fitsqimage = new QImage((uchar*)fitsdata, imgwidth, imgheight, QImage::Format_Indexed8);
    } else {
        fitsqimage = new QImage((uchar*)scaledfitsdata, imgwidth, imgheight, QImage::Format_Indexed8);
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

void ccd_client::saveBLOB(IBLOB *bp) {
    FILE *fp;
    long indFITS;
    char *fitsdata = nullptr;

    fp= fopen("GuiderCAM_Probe.fits", "wb");
    if (fp == NULL) {
        qDebug() << "Cannot open FITS file for storing the camera image ...";
        perror("Error: ");
        return;
    } else {
            fitsdata = (static_cast<char *>(bp->blob));
        for (indFITS = 0; indFITS < bp->bloblen; indFITS++) {
            fprintf(fp,static_cast<char *>(fitsdata));
            fitsdata++;
        }
        fclose(fp);
        qDebug() << "First image from guidercam written...";
    }
}
