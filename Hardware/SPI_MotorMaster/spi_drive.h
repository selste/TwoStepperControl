#ifndef SPI_DRIVE_H
#define SPI_DRIVE_H
#include <wiringPiSPI.h>
#include <qstring.h>

class SPI_Drive {
public:
    SPI_Drive(short);
    int spidrGetFD(void);
    void spidrReceiveCommand(QString);

private:
    int SPIChannel; //0 or 1, there is 2 channels on the Pi
    QString *parameter;
    int fileDesc; // filedescriptor for opening a channel. -1 if failed ...
};

#endif // SPI_DRIVE_H
