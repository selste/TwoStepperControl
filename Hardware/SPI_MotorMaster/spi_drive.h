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
