#include "usb_communications.h"
#include <libusb-1.0/libusb.h>
#include <QDebug>
#include <QElapsedTimer>

//-------------------------------------------------------------------------------
// open USB devices with given VID and send one byte - the <ACK> character;
// the response of the device to the <ACK> = 0x06 is recorded as a QString "startupResponse...

usbCommunications::usbCommunications(int whichVID) {
    int retVal; // for return values of libusb - calls
    int noOfBytesWritten, noOfBytesSent; // just to check the number of bytes sent
    short numberOfFoundDevices = 0, deviceCounter;
    libusb_device_descriptor desc;
    ssize_t idx;
    unsigned char *replyData = new unsigned char[64];
    int noOfBytesRead;

    this->commandData[0]= new unsigned char[32];
    this->commandData[1]= new unsigned char[32];
    this->theVID = whichVID; // store vendor id and product id in the class
    this->dataReceived[0] = new QString();
    this->dataReceived[1] = new QString();
    retVal = libusb_init(&usbContext); // initialize the library for the current session
    if(retVal < 0) {
        this->initErr = true;
        this->usbConnAvailable = false;
        return;
    } else {
        this->usbConnAvailable = true;
    }
    libusb_set_debug(this->usbContext, 3); // set verboseness level to 3
    this->devCnt = libusb_get_device_list(this->usbContext, &deviceList); // get the list of devices
    if(this->devCnt < 0) {
        this->gotDeviceList = false;
        this->usbConnAvailable = false;
        return;
    } else {
        this->gotDeviceList = true;
        this->usbConnAvailable = true;
    }
    // now list devices and their properties; open all teensy devices based on the VID given
    for (idx = 0; idx < this->devCnt; ++idx) {
        libusb_device *device = deviceList[idx];
        retVal = libusb_get_device_descriptor(device, &desc);
        if (desc.idVendor == whichVID) { // this is the adafruit device descriptor
            numberOfFoundDevices++;
            if (numberOfFoundDevices == 1) {
                libusb_open(device, &deviceHandles[0]);
                if(deviceHandles[0] == NULL) {
                    this->usbDeviceIsOpen = false;
                    this->usbConnAvailable = false;
                    return;
                } else {
                    this->usbDeviceIsOpen = true;
                    this->usbConnAvailable = true;
                }
            } else {
                libusb_open(device, &deviceHandles[1]);
                if(deviceHandles[1] == NULL) {
                    this->usbDeviceIsOpen = false;
                    this->usbConnAvailable = false;
                    return;
                } else {
                    this->usbDeviceIsOpen = true;
                    this->usbConnAvailable = true;
                }
            }
        }
    }
    if(libusb_kernel_driver_active(deviceHandles[0], 0) == 1) { // find out if kernel driver is attached
        this->kernelDriverActive = true;
    if(libusb_detach_kernel_driver(deviceHandles[0], 0) == 0) //detach it
        this->kernelDriverActive = false;
    }
    if(libusb_kernel_driver_active(deviceHandles[1], 0) == 1) { // find out if kernel driver is attached
        this->kernelDriverActive = true;
    if(libusb_detach_kernel_driver(deviceHandles[1], 0) == 0) //detach it
        this->kernelDriverActive = false;
    }
    qDebug() << "claiming interfaces";
    this->commandData[0][0]=0x06; // just send <ACK> to the device; the device can answer with a string which is stored as
                                   // "startupResponse". This can be a human readable device name, for instance
    noOfBytesSent = 1;
    bzero(replyData,64);
    replyData[0] = '\0';
    for (deviceCounter = 0; deviceCounter < 2; deviceCounter++) {
        retVal = libusb_bulk_transfer(deviceHandles[deviceCounter],
                 (0x03 | LIBUSB_ENDPOINT_OUT), this->commandData[0], noOfBytesSent,
                 &noOfBytesWritten, 500); // device endpoints can be detemined using lsusb - see comment below
        qDebug() << "Wrote command with result: "  << libusb_error_name(retVal);
        if (retVal == 0 && noOfBytesWritten == noOfBytesSent) {
            this->writeError = false;
        } else {
            this->writeError = true;
            this->usbConnAvailable = false;
        }
        retVal = libusb_bulk_transfer(deviceHandles[deviceCounter], (0x84 | LIBUSB_ENDPOINT_IN), replyData, 64, &noOfBytesRead, 1000); // finding out endpoints is done by running lsusb -v -d VID:PID
        if (strcmp((const char*)replyData,"TSC_RA") == 0) {
            this->indexForRA = deviceCounter;
            qDebug() << "RA drive assigned with id: " << deviceCounter;
        }
        if (strcmp((const char*)replyData,"TSC_DE") == 0) {
            this->indexForDecl = deviceCounter;
            qDebug() << "Decl drive assigned with id: " << deviceCounter;
        }
        bzero(replyData,64);
    }
    libusb_free_device_list(this->deviceList, 1); // free the list and unref the devices in it
    qDebug() << "usb constructor successful.";
}



//--------------------------------------------------------------------------------------------------
// shutdown the connection and free the USB device
void usbCommunications::closeUSBConnection(void) {

    libusb_release_interface(this->deviceHandles[0], 0); // release the claimed interface
    libusb_release_interface(this->deviceHandles[1], 0); // release the claimed interface
    libusb_close(this->deviceHandles[0]); // close the device we opened
    libusb_close(this->deviceHandles[1]); // close the device we opened
    libusb_exit(this->usbContext);
    this->usbConnAvailable = false;
    delete this->dataReceived[0];
    delete this->dataReceived[1];
    delete this->startupResponse;
}

//----------------------------------------------------------------------------------------------------
// get error flags for USB communication

bool usbCommunications::getUSBErrs(usbState errStat) {
    switch (errStat) {
        case open: return this->usbDeviceIsOpen; break;
        case avail: return this->usbConnAvailable; break;
        case init: return this->initErr; break;
        case devListAvail: return this->gotDeviceList; break;
        case kernelDrvr: return this->kernelDriverActive; break;
        case claimed: return this->interfaceClaimed; break;
        case writeErr: return this->writeError; break;
        case released: return this->interfaceReleased; break;
        case readErr: return this->readError; break;
    }
    return false;
}

// -----------------------------------------------------------------------------------------------------
// send a string to the microcontroller via USB; as we use bulk transfer, it should not be bigger than 64 bytes

bool usbCommunications::sendCommand(QString theCmd, bool isRA) {
    int cmdLen, cntr, noOfBytesWritten;
    int retVal;
    short idx;

    this->deleteResponse(isRA);
    if (isRA == true) {
        idx = this->indexForRA;
    } else {
        idx = this->indexForDecl;
    }
    cmdLen = theCmd.length();
    for (cntr = 0; cntr < cmdLen; cntr++) {
        this->commandData[idx][cntr] = (unsigned char)(theCmd.at(cntr).toLatin1());
    } // converted the QString to unsigned char ...

    retVal = libusb_bulk_transfer(this->deviceHandles[idx], (0x03 | LIBUSB_ENDPOINT_OUT), this->commandData[idx], cmdLen, &noOfBytesWritten, 1000); // finding out endpoints is done by running lsusb -v -d VID:PID
    if(retVal == 0 && noOfBytesWritten == cmdLen) {
        this->writeError = false;
    } else {
        this->writeError = true;
        this->usbConnAvailable = false; // if a write error occurs, it is assumed that the connection is broken
        qDebug() << "Write error!";
    }
    this->receiveReply(isRA);
    this->commandData[idx][0] ='\0';
    return this->writeErr;
    // return values of libusb_bulk_transfer are
    //enum libusb_error {LIBUSB_SUCCESS = 0, LIBUSB_ERROR_IO = -1, LIBUSB_ERROR_INVALID_PARAM = -2,
    // LIBUSB_ERROR_ACCESS = -3, LIBUSB_ERROR_NO_DEVICE = -4, LIBUSB_ERROR_NOT_FOUND = -5, LIBUSB_ERROR_BUSY = -6,
    // LIBUSB_ERROR_TIMEOUT = -7, LIBUSB_ERROR_OVERFLOW = -8, LIBUSB_ERROR_PIPE = -9, LIBUSB_ERROR_INTERRUPTED = -10,
    // LIBUSB_ERROR_NO_MEM = -11, LIBUSB_ERROR_NOT_SUPPORTED = -12, LIBUSB_ERROR_OTHER = -99
}

//----------------------------------------------------------------------------------------------------------
// receive data via USB; a timeout is implemented

bool usbCommunications::receiveReply(bool isRA) {
    int retVal, noOfBytesRead, cntr;
    unsigned char *replyData = new unsigned char[64];
    QElapsedTimer *waitingTimer;
    bool readLoopDone = false;
    short idx;

    if (isRA == true) {
        idx = this->indexForRA;
    } else {
        idx = this->indexForDecl;
    }
    waitingTimer = new QElapsedTimer();
    waitingTimer->start();
    do {
        retVal = libusb_bulk_transfer(this->deviceHandles[idx], (0x84 | LIBUSB_ENDPOINT_IN), replyData, 64, &noOfBytesRead, 250); // finding out endpoints is done by running lsusb -v -d VID:PID
        if (noOfBytesRead > 0) {
            readLoopDone = true;
        }
        if (waitingTimer->elapsed() > 250) {
            readLoopDone = true;
        } // i had trouble reading data on first try, this also depends on the device; so i try reading until the timeout comes
    } while (readLoopDone == false);
    delete waitingTimer;
    this->dataReceived[idx]->clear();
    if (noOfBytesRead > 0) {
        for (cntr = 0; cntr < noOfBytesRead; cntr++) {
            this->dataReceived[idx]->append((QChar)replyData[cntr]);
        }
    }
    delete replyData;
    if (retVal == 0) {
        this->readError = false;
        return true;
    } else {
        if (retVal != -7) { // timeout errors are ignored
            this->readError = true;
            return false;
        } else {
            this->dataReceived[idx]->clear();
            this->dataReceived[idx]->append("Timeout from USB");
            this->readError = false;
            return true;
        }
    }
}

//------------------------------------------------------------------------------------------------------------
// retrieve the device response from the class

QString usbCommunications::getReply(bool isRA) {
    short idx;
    QString reply;

    if (isRA == true) {
        idx = this->indexForRA;
    } else {
        idx = this->indexForDecl;
    }
    reply = QString(this->dataReceived[idx]->data());
    this->dataReceived[idx]->clear();
    return reply;
}

//------------------------------------------------------------------------------------------------------------
// just flush the reponse string after receiving it to make sure no old repsonses lurk around

void usbCommunications::deleteResponse(bool isRA) {
    short idx;

    if (isRA == true) {
        idx = this->indexForRA;
    } else {
        idx = this->indexForDecl;
    }
    this->dataReceived[idx]->clear();
}
