#include "indidevapi.h"
#include "indicom.h"
#include "indibase/baseclient.h"
#include <QString>
#include <QImage>
#include <QPixmap>

class alccd5_client : public INDI::BaseClient {
 public:
    alccd5_client();
    ~alccd5_client();
    void takeExposure(int);
    bool setINDIServer(QString, int);
    QPixmap* getScaledPixmapFromCamera(void);
    bool newImageArrived(void);
    void newImageUsedAsPixmap(void);

protected:
    virtual void newDevice(INDI::BaseDevice *dp);
    virtual void removeDevice(INDI::BaseDevice *dp) {}
    virtual void newProperty(INDI::Property *property);
    virtual void removeProperty(INDI::Property *property) {}
    virtual void newBLOB(IBLOB *bp);
    virtual void newSwitch(ISwitchVectorProperty *svp) {}
    virtual void newNumber(INumberVectorProperty *nvp);
    virtual void newMessage(INDI::BaseDevice *dp, int messageID);
    virtual void newText(ITextVectorProperty *tvp) {}
    virtual void newLight(ILightVectorProperty *lvp) {}
    virtual void serverConnected() {}
    virtual void serverDisconnected(int exit_code) {}

private:
   INDI::BaseDevice * alccd5;
   QImage* fitsqimage;
   QPixmap* displayPMap;
   int imgwidth;
   int imgheight;
   bool newCameraImageAvailable;
};
