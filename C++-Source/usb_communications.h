#include <libusb-1.0/libusb.h>
#include <QString>

class usbCommunications {
public:
    enum usbState {init, avail, devListAvail, open, kernelDrvr, claimed, writeErr, released, readErr};
    usbCommunications(int);
    void closeUSBConnection(void);
    bool getUSBErrs(usbState);
    bool sendCommand(QString,bool);
    bool receiveReply(bool);
    QString getReply(bool);
    void deleteResponse(bool);

private:
    libusb_device **deviceList; //pointer to pointer of device, used to retrieve a list of devices
    libusb_device_handle *deviceHandles[2]; // a device handle
    libusb_context *usbContext = NULL; //a libusb session
    ssize_t devCnt; //holding number of devices in list
    unsigned char *commandData[2]; // an array holding the command to be sent to the USB device
    int theVID;
    short indexForRA;
    short indexForDecl;
    bool initErr = false;
    bool usbConnAvailable = true;
    bool gotDeviceList = false;
    bool usbDeviceIsOpen = false;
    bool kernelDriverActive = false;
    bool interfaceClaimed = false;
    bool writeError = false;
    bool interfaceReleased = false;
    bool readError = false;
    QString* dataReceived[2];
    QString* startupResponse;
};
