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
// this class handles reading FITS files from an INDI server for autoguiding

#include <indidevapi.h>
#include <indicom.h>
#include <baseclient.h>
#include <QString>
#include <QImage>
#include <QPixmap>
#include <QVector>
#include <QObject>
#include <QCoreApplication>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

class ccd_client:public QObject, public INDI::BaseClient {
    Q_OBJECT
 public:
    ccd_client(bool);
    ~ccd_client();
    void takeExposure(int);
    void sendGain(int);
    bool setINDIServer(QString, int);
    QString* getINDIServerMessage(void);
    void sayGoodbyeToINDIServer(void);
    bool getCCDParameters(void);
    void setStoreImageFlag(bool);
    void setCameraName(QString);
    void disconnectFromServer(void);
    bool probeForCCD(void);
    bool cameraGainAvailable(void);

protected:
    virtual void newDevice(INDI::BaseDevice *dp);
    virtual void removeDevice(INDI::BaseDevice *dp) {}
    virtual void newProperty(INDI::Property *property);
    virtual void removeProperty(INDI::Property *property) {}
    virtual void newBLOB(IBLOB *bp);
    virtual void newSwitch(ISwitchVectorProperty *svp) {}
    virtual void newNumber(INumberVectorProperty *nvp) {}
    virtual void newMessage(INDI::BaseDevice *dp, int messageID);
    virtual void newText(ITextVectorProperty *tvp) {}
    virtual void newLight(ILightVectorProperty *lvp) {}
    virtual void serverConnected() {}
    virtual void serverDisconnected(int exit_code) {}

private:
   QString *ccdINDIName; // name of the camera in INDI lingo
   INDI::BaseDevice *ccd; // the camera device
   double pixSizeX; // physical size of one pixel in x-direction
   double pixSizeY; // physical size of one pixel in y-direction
   double frameSizeX; // number of pixels in x-direction
   double frameSizeY; // number of pixles in y-direction
   double bitsPerPixel; // depth of the camera
   bool cameraHasGain; // some cameras can set gain, some cannot ...
   QImage* fitsqimage; // a qimage, generated form raw FITS data
   char *scaledfitsdata; // in case of 16 bit data, this one holds the scaled 8 bit image
   QPixmap* displayPMap; // a qpixmap, generated for GUI display form the qimage
   bool newCameraImageAvailable; // name says it all
   QVector<QRgb> *myVec; // a vector for grayscale conversion
   QString *serverMessage; // a string for holding data non-image data from the INDI server
   INumberVectorProperty *ccd_exposure = NULL; // an INDI data structure on exposure time
   INumberVectorProperty *ccd_gain = NULL; // an INDI data structure on camera gain
   long expcounter; // a counter for exposures
   bool storeCamImages; // a boolean for handling storage of images to the SD card
   short simulatorCounter; // a helper for debugging
   bool isAProbeImage; // a flag that causes storage rather than further processing of image data
   bool isMainCCD;
   void saveBLOB(IBLOB*); // saves a BLOB to a FITS image

signals:
   void imageAvailable(QPixmap*); // emitted when an image is available
   void messageFromINDIAvailable(void); // emitted when a message form INDI is available
};
