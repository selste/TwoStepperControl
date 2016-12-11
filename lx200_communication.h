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
    qint64 getDataFromSerialPort(void);
private:
    QSerialPort rs232port;
    bool portIsUp;
    char dataRead[5000];
};

#endif // LX200_COMMUNICATION_H
