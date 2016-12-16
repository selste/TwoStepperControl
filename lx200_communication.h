#ifndef LX200_COMMUNICATION_H
#define LX200_COMMUNICATION_H

#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include <QByteArray>
#include <QObject>
#include <QString>

class lx200_communication {
public:
    lx200_communication(void);
    ~lx200_communication(void);
    bool getPortState(void);
    qint64 getDataFromSerialPort(void);
    void shutDownPort(void);
    void openPort(void);

private:
    QSerialPort rs232port;
    QString *replyStrLX;
    bool portIsUp;
    QByteArray *serialData;
    bool handleBasicLX200Protocol(QString);
    void assembleDeclinationString(void);
    void assembleRAString(void);
    struct LX200CommandStruct {
        QString getDecl;
        QString getRA;
        QString getHiDef;
        QString stopMotion;
        QString slewRA;
        QString slewDecl;
        QString slewPossible;
        QString syncCommand;
        QString moveEast;
        QString moveWest;
        QString moveNorth;
        QString moveSouth;
        QString stopMoveEast;
        QString stopMoveWest;
        QString stopMoveNorth;
        QString stopMoveSouth;
        QString setCenterSpeed;
        QString setGuideSpeed;
        QString setFindSpeed;
        QString setGOTOSpeed;
    };
    struct LX200CommandStruct LX200Commands;
};

#endif // LX200_COMMUNICATION_H
