#include "usb_communications.h"
#include <libusb-1.0/libusb.h>
#include <QDebug>
#include <QElapsedTimer>

//-------------------------------------------------------------------------------
// open a USB device with VID and PID and send one byte - the <ACK> character;
// the response of the device to the <ACK> = 0x06 is recorded as a QString "startupResponse...

usbCommunications::usbCommunications(int whichVID, int whichPID) {
    int retVal; // for return values of libusb - calls
    int noOfBytesWritten, noOfBytesSent; // just to check the number of bytes sent

    this->theVID = whichVID; // store vendor id and product id in the class
    this->thePID = whichPID;

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
    this->deviceHandle = libusb_open_device_with_vid_pid(this->usbContext, this->theVID, this->thePID); // pass VID and PID for the USB device
    if(this->deviceHandle == NULL) {
        this->usbDeviceIsOpen = false;
        this->usbConnAvailable = false;
    } else {
        this->usbDeviceIsOpen = true;
        this->usbConnAvailable = true;
    }
    libusb_free_device_list(this->deviceList, 1); // free the list and unref the devices in it
    if(libusb_kernel_driver_active(this->deviceHandle, 0) == 1) { // find out if kernel driver is attached
        this->kernelDriverActive = true;
    if(libusb_detach_kernel_driver(this->deviceHandle, 0) == 0) //detach it
        this->kernelDriverActive = false;
    }
    retVal = libusb_claim_interface(this->deviceHandle, 0); // claim interface 0 (the first) of device
    if(retVal < 0) {
        this->interfaceClaimed = false;
        this->usbConnAvailable = false;
        return;
    } else {
        this->interfaceClaimed = true;
        this->usbConnAvailable = true;
    }
    this->commandData[0]=0x06; // just send <ACK> to the device; the device can answer with a string which is stored as
                               // "startupResponse". This can be a human readable device name, for instance
    noOfBytesSent = 1;
    retVal = libusb_bulk_transfer(this->deviceHandle, (0x02 | LIBUSB_ENDPOINT_OUT), this->commandData, noOfBytesSent, &noOfBytesWritten, 0); // device endpoints can be detemined using lsusb - see comemnt below
    if(retVal == 0 && noOfBytesWritten == noOfBytesSent) {
        this->writeError = false;
    } else {
        this->writeError = true;
        this->usbConnAvailable = false;
    }
    if (this->usbConnAvailable == true) {
    }
    this->dataReceived = new QString(); // initialize a string for holding the devices response
    this->receiveReply();
    this->startupResponse = new QString(this->dataReceived->toLatin1()); // as only <ACK> was sent, this repsonse should be the dvice name ...
    this->dataReceived->clear();
}

//--------------------------------------------------------------------------------------------------
// shutdown the connection and free the USB device
void usbCommunications::closeUSBConnection(void) {
    int retVal;

    retVal = libusb_release_interface(this->deviceHandle, 0); // release the claimed interface
    if(retVal != 0) {
        this->interfaceReleased = false;
    return;
    }

    libusb_close(this->deviceHandle); // close the device we opened
    libusb_exit(this->usbContext);
    this->usbConnAvailable = false;
    delete this->dataReceived;
    delete this->startupResponse;
}

//--------------------------------------------------------------------------------------------------
// get VID or PID for attached device; return 0 if no connection is available

int usbCommunications::getUSBIDs(usbIDs theID) {
    if (this->usbConnAvailable == false) {
        return 0;
    }
    if (theID == isVID) {
        return this->theVID;
    } else {
        return this->thePID;
    }
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
// send a string to the microcontroller via USB; as we use bul transfer, it should not be bigger than 64 bytes

bool usbCommunications::sendCommand(QString theCmd) {
    int cmdLen, cntr, noOfBytesWritten;
    int retVal;

    cmdLen = theCmd.length();
    for (cntr = 0; cntr < cmdLen; cntr++) {
        this->commandData[cntr] = (unsigned char)(theCmd.at(cntr).toLatin1());
    } // converted the QString to unsigned char ...

    retVal = libusb_bulk_transfer(this->deviceHandle, (0x02 | LIBUSB_ENDPOINT_OUT), this->commandData, cmdLen, &noOfBytesWritten, 1000); // finding out endpoints is done by running lsusb -v -d VID:PID
    if(retVal == 0 && noOfBytesWritten == cmdLen) {
        this->writeError = false;
    } else {
        this->writeError = true;
        this->usbConnAvailable = false; // if a write error occurs, it is assumed that the connection is broken
    }
    this->receiveReply();
    return this->writeErr;
    // return values of libusb_bulk_transfer are
    //enum libusb_error {LIBUSB_SUCCESS = 0, LIBUSB_ERROR_IO = -1, LIBUSB_ERROR_INVALID_PARAM = -2,
    // LIBUSB_ERROR_ACCESS = -3, LIBUSB_ERROR_NO_DEVICE = -4, LIBUSB_ERROR_NOT_FOUND = -5, LIBUSB_ERROR_BUSY = -6,
    // LIBUSB_ERROR_TIMEOUT = -7, LIBUSB_ERROR_OVERFLOW = -8, LIBUSB_ERROR_PIPE = -9, LIBUSB_ERROR_INTERRUPTED = -10,
    // LIBUSB_ERROR_NO_MEM = -11, LIBUSB_ERROR_NOT_SUPPORTED = -12, LIBUSB_ERROR_OTHER = -99
}

//----------------------------------------------------------------------------------------------------------
// receive data via USB; a timeout is implemented

bool usbCommunications::receiveReply(void) {
    int retVal, noOfBytesRead, cntr;
    unsigned char *replyData = new unsigned char[64];
    QElapsedTimer *waitingTimer;
    bool readLoopDone = false;

    waitingTimer = new QElapsedTimer();
    waitingTimer->start();
    do {
        retVal = libusb_bulk_transfer(this->deviceHandle, (0x83 | LIBUSB_ENDPOINT_IN), replyData, 64, &noOfBytesRead, 500); // finding out endpoints is done by running lsusb -v -d VID:PID
        if (noOfBytesRead > 0) {
            readLoopDone = true;
        }
        if (waitingTimer->elapsed() > 500) {
            readLoopDone = true;
        } // i had trouble reading data on first try, this also depends on the device; so i try reading until the timeout comes
    } while (readLoopDone == false);

    if (noOfBytesRead > 0) {
        this->dataReceived->clear();
        for (cntr = 0; cntr < noOfBytesRead; cntr++) {
            this->dataReceived->append((QChar)replyData[cntr]);
        }
    }
    delete replyData;
    delete waitingTimer;
    if (retVal == 0) {
        this->readError = false;
        return true;
    } else {
        if (retVal != -7) { // timeout errors are ignored
            this->readError = true;
            return false;
        } else {
            this->dataReceived->clear();
            this->dataReceived->append("Timeout from USB");
            this->readError = false;
            return true;
        }
    }
}

//------------------------------------------------------------------------------------------------------------
// retrieve the device response from the class

QString* usbCommunications::getReply(void) {
    return this->dataReceived;
}

//------------------------------------------------------------------------------------------------------------
// retrieve the device name = response to <ACK> from the class

QString* usbCommunications::getInitResponse(void) {
    return this->startupResponse;
}
//------------------------------------------------------------------------------------------------------------
// just flush the reponse string after receiving it to make sure no old repsonses lurk around

void usbCommunications::deleteResponse(void) {
    this->dataReceived->clear();
    this->startupResponse->clear();
}
