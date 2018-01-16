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
    char bytecmd[32];

    this->parameter->clear();
    this->parameter->append(cmd);
    strncpy(bytecmd, (const char*)(this->parameter->toLatin1()),30);
    len = strlen(bytecmd);
    bytecmd[len] = 0x00;
    len++;
    wiringPiSPIDataRW(this->SPIChannel, (unsigned char*)(bytecmd), len);
    this->muprocReply=bytecmd[1];

}

//--------------------------------------------------------------

int SPI_Drive::spidrGetFD(void) {
    return this->fileDesc;
}

//--------------------------------------------------------------

char SPI_Drive::getResponse(void) {
    return this->muprocReply;
}
