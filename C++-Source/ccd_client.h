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

class ccd_client:public QObject, public INDI::BaseClient { Q_OBJECT
 public:
    ccd_client();
    ~ccd_client();
    void takeExposure(int,bool);
    bool setINDIServer(QString, int);
    QString* getINDIServerMessage(void);
    void sayGoodbyeToINDIServer(void);
    bool getCCDParameters(bool);
    void setStoreImageFlag(bool);
    void disconnectFromServer(void);
    bool probeForCCD(short);
    QString* getINDIName(short);
    void setIdxOfCCDs(short,bool);
    int getNoOfProperties(bool);
    QString* getPropertyName(bool, int);
    QString* getPropertyLabel(bool, int);
    QString* getPropertyGroup(bool, int);
    INDI_PROPERTY_TYPE getPropertyType(bool, int);
    QString* getTextPropertyItemLabel(bool, int, int);
    QString* getNumberPropertyItemLabel(bool, int, int);
    QString* getSwitchPropertyItemLabel(bool, int, int);
    QString* getLightPropertyItemLabel(bool, int, int);
    QString* getBLOBPropertyItemLabel(bool, int, int);
    bool getRWPermission(bool, int, bool);
    INumber getCurrentNumber(bool, int, int);
    QString* getCurrentText(bool, int, int);
    ISState getCurrentSwitch(bool, int, int);
    void setCurrentNumberValue(bool, int, int, double);
    void setCurrentTextValue(bool, int, int, QString);
    int getNoOfValuesInProperty(bool, int);
    ISRule getSwitchRules(bool, int);
    void setCurrentSwitch(bool, int, int, bool);
    void sendCurrentNumberValueCompound(void);
    void loadINDIConfigFile(bool);
    void saveINDIConfigFile(bool);

protected:
    virtual void newDevice(INDI::BaseDevice *dp);
    virtual void removeDevice(INDI::BaseDevice *dp);
    virtual void newProperty(INDI::Property *property);
    virtual void removeProperty(INDI::Property *property);
    virtual void newBLOB(IBLOB *bp);
    virtual void newSwitch(ISwitchVectorProperty *svp);
    virtual void newNumber(INumberVectorProperty *nvp);
    virtual void newMessage(INDI::BaseDevice *dp, int messageID);
    virtual void newText(ITextVectorProperty *tvp);
    virtual void newLight(ILightVectorProperty *lvp);
    virtual void serverConnected() {}
    virtual void serverDisconnected(int exit_code);

private:
   QString *ccdINDIName[2]; // name of the cameras in INDI lingo
   QString *guiderCCDName;
   INDI::BaseDevice *ccd[2]; // the camera device
   QString* propertyName;
   QString* propertyGroup;
   QString* propertyLabel;
   bool firstDeviceConnected = false;
   bool secondDeviceConnected = false; // set to true if cameras are identified
   short idxOfGuiderCCD = 0;
   short idxOfMainCCD = 0;
   short noOfCams = 0; // number of cameras connected
   double pixSizeX; // physical size of one pixel in x-direction
   double pixSizeY; // physical size of one pixel in y-direction
   double frameSizeX; // number of pixels in x-direction
   double frameSizeY; // number of pixles in y-direction
   double bitsPerPixel; // depth of the camera
   double cameraDepth[2];
   QImage* fitsqimage; // a qimage, generated form raw FITS data
   QImage* mainCamImage; // a QImage generated form the main camera
   char *scaledfitsdata[2]; // in case of 16 bit data, this one holds the scaled 8 bit image
   QPixmap* displayPMap; // a qpixmap, generated for GUI display form the qimage for the guider
   QPixmap* mainCCDPMap; // the same for the main camera
   bool newCameraImageAvailable; // name says it all
   QVector<QRgb> *myVec; // a vector for grayscale conversion
   QString *serverMessage; // a string for holding data non-image data from the INDI
   INDI::Property *propertyArray[500][2]; // an array holding all properties of the 2 cameras
   int noOfProperties[2]; // a counter for the number of properties that arrived
   INumberVectorProperty *ccd_exposure = NULL; // an INDI data structure on exposure time
   INumberVectorProperty *ccd_gain = NULL; // an INDI data structure on camera gain
   INumberVectorProperty *ccd_params = NULL;
   INumberVectorProperty *currentNumberVector = NULL;
   ITextVectorProperty *currentTextVector = NULL;
   ISwitchVectorProperty *currentSwitchVector = NULL;
   ILightVectorProperty *currentLightVector = NULL;
   IBLOBVectorProperty *currentBLOBVector = NULL;
   QString* tprop;
   INumber nprop;

   long expcounter; // a counter for exposures
   bool storeCamImages; // a boolean for handling storage of images to the SD card
   short simulatorCounter; // a helper for debugging
   bool isAProbeImage; // a flag that causes storage rather than further processing of image data

signals:
   void imageAvailable(QPixmap*); // emitted when an image is available
   void mainCCDImageAvailable(QPixmap*); // emitted when an image from the main camera is available
   void messageFromINDIAvailable(void); // emitted when a message form INDI is available
   void newPropertyListArrived(void); // emitted when properties come - porperties need to be reloaded in main window
   void numberSetOnServerMainCCD(void);
   void textSetOnServerMainCCD(void);
   void switchSetOnServerMainCCD(void);
   void numberSetOnServerGuiderCCD(void);
   void textSetOnServerGuiderCCD(void);
   void switchSetOnServerGuiderCCD(void);
   void saveFunctionNotAvailable(void);
};
