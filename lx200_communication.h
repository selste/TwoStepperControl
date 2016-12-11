#ifndef LX200_COMMUNICATION_H
#define LX200_COMMUNICATION_H

#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include <QByteArray>
#include <QObject>

class lx200_communication {
public:
    lx200_communication(void);
    ~lx200_communication(void);
    bool getPortState(void);
private:
    QSerialPort rs232port;
    bool portIsUp;
    QByteArray  rs232ReadData;
    QTimer      RS232Timer;
};

#endif // LX200_COMMUNICATION_H
