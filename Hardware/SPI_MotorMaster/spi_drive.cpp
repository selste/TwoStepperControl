#include "spi_drive.h"

//--------------------------------------------------------------

SPI_Drive::SPI_Drive(short channel) {
    int fd = 0;

    this->SPIChannel = channel;
    fd = wiringPiSPISetup(this->SPIChannel, 500000);
    this->fileDesc = fd;
    parameter = new QString();
}

//--------------------------------------------------------------

void SPI_Drive::spidrReceiveCommand(QString cmd) {
    delete parameter;
    parameter = new QString(cmd);
}

//--------------------------------------------------------------

int SPI_Drive::spidrGetFD(void) {
    return this->fileDesc;
}

//--------------------------------------------------------------
