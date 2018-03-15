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

#ifndef SPI_DRIVE_H
#define SPI_DRIVE_H
#include <wiringPiSPI.h>
#include <qstring.h>

class SPI_Drive {
public:
    SPI_Drive(short);
    int spidrGetFD(void);
    void spidrReceiveCommand(QString);
    char getResponse(void);

private:
    int SPIChannel;   //0 or 1, there is 2 channels on the Pi
    char muprocReply; // the microcontroller answers with A, D or R in dependence of the stepper driver used.
                      // A is for the A4988, D is for DRV8825 and R is for the RAPS128
    QString *parameter;
    int fileDesc; // filedescriptor for opening a channel. -1 if failed ...
};

#endif // SPI_DRIVE_H
