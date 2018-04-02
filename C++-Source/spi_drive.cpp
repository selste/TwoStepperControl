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
