#include <indidevapi.h>
#include <indicom.h>
#include <baseclient.h>
#include <QString>
#include <QImage>
#include <QPixmap>
#include <QVector>
#include <QObject>
#include <QCoreApplication>

class ccd_client:public QObject, public INDI::BaseClient {
    Q_OBJECT
 public:
    ccd_client();
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
   INDI::BaseDevice *ccd;
   double pixSizeX;
   double pixSizeY;
   double frameSizeX;
   double frameSizeY;
   double bitsPerPixel;
   QImage* fitsqimage;
   QPixmap* displayPMap;
   bool newCameraImageAvailable;  
   QVector<QRgb> *myVec;
   QString *serverMessage;
   INumberVectorProperty *ccd_exposure = NULL;
   INumberVectorProperty *ccd_gain = NULL;
   long expcounter;
   bool storeCamImages;
   short simulatorCounter; // a helper for debugging

signals:
   void imageAvailable(QPixmap*);
   void messageFromINDIAvailable(void);
};
