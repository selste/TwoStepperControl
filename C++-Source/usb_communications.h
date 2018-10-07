#ifndef USB_COMMUNICATIONS_H
#define USB_COMMUNICATIONS_H

#endif // USB_COMMUNICATIONS_H

#include <libusb-1.0/libusb.h>
#include <QString>

class usbCommunications {
public:
    enum usbIDs {isVID, isPID};
    enum usbState {init, avail, devListAvail, open, kernelDrvr, claimed, writeErr, released, readErr};
    usbCommunications(int, int);
    void closeUSBConnection(void);
    int getUSBIDs(usbIDs);
    bool getUSBErrs(usbState);
    bool sendCommand(QString);
    bool receiveReply(void);
    QString* getReply(void);
    QString* getInitResponse(void);
    void deleteResponse(void);


private:
    libusb_device **deviceList; //pointer to pointer of device, used to retrieve a list of devices
    libusb_device_handle *deviceHandle; // a device handle
    libusb_context *usbContext = NULL; //a libusb session
    ssize_t devCnt; //holding number of devices in list
    unsigned char *commandData = new unsigned char[32]; // an array holding the command to be sent to the USB device
    int theVID;
    int thePID;
    bool initErr = false;
    bool usbConnAvailable = true;
    bool gotDeviceList = false;
    bool usbDeviceIsOpen = false;
    bool kernelDriverActive = false;
    bool interfaceClaimed = false;
    bool writeError = false;
    bool interfaceReleased = false;
    bool readError = false;
    QString* dataReceived;
    QString* startupResponse;
};
