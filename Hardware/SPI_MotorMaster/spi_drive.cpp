#include "spi_drive.h"
#include <string.h>
#include <QDebug>

//--------------------------------------------------------------

SPI_Drive::SPI_Drive(short channel) {
    int fd = 0;

    this->SPIChannel = channel;
    fd = wiringPiSPISetup(this->SPIChannel, 500000);
    this->fileDesc = fd;
    this->parameter = new QString();
}

//--------------------------------------------------------------

void SPI_Drive::spidrReceiveCommand(QString cmd) {
    int len;

    this->parameter->clear();
    this->parameter->append(cmd);
    strncpy(this->bytecmd, (const char*)(this->parameter->toLatin1()),30);
    len = strlen(this->bytecmd);
    this->bytecmd[len] = 0x00;
    wiringPiSPIDataRW(this->SPIChannel, (unsigned char*)(this->bytecmd), (len+1));
}

//--------------------------------------------------------------

int SPI_Drive::spidrGetFD(void) {
    return this->fileDesc;
}

//--------------------------------------------------------------
