#ifndef LX200_COMMUNICATION_H
#define LX200_COMMUNICATION_H

#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include <QByteArray>
#include <QObject>
#include <QString>
#include <QCoreApplication>

class lx200_communication:public QObject {
    Q_OBJECT
public:
    lx200_communication(void);
    ~lx200_communication(void);
    bool getPortState(void);
    qint64 getDataFromSerialPort(void);
    void shutDownPort(void);
    void openPort(void);
    double getReceivedCoordinates(short); // 0 for RA, 1 for Decl - retrieve data conveyed from LX
    QString* getLX200Command(void);
    QString* getLX200Response(void);
    QString* getLX200ResponseRA(void);
    QString* getLX200ResponseDecl(void);
    void setNumberFormat(bool);

private:
    QSerialPort rs232port;
    QString *replyStrLX;
    QString *assembledString;
    QString *msgRAString;
    QString *msgDeclString;
    QString *incomingCommand;
    QString *subCmd;
    bool portIsUp;
    QByteArray *serialData;
    double receivedRAFromLX;
    double receivedDeclFromLX;
    bool handleBasicLX200Protocol(QString);
    bool gotRACoordinates;
    bool gotDeclCoordinates;
    bool sendSimpleCoordinates; // use either ddd.mm or ddd.mm.ss
    void assembleDeclinationString(void);
    void assembleRAString(void);
    void sendCommand(short what);
    struct LX200CommandStruct {
        QString slewRA;
        QString slewDecl;
        QString slewPossible;
        QString syncCommand;
        QString getDecl;
        QString getRA;
        QString getHiDef;
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
        QString stopMotion;
    };
    struct LX200CommandStruct LX200Commands;
signals:
    void RS232CommandReceived(void);
    void RS232RASent(void);
    void RS232DeclSent(void);
    void RS232CommandSent(void);
    void RS232slew(void);
    void RS232sync(void);
    void RS232stopMotion(void);
    void RS232stopMoveEast(void);
    void RS232stopMoveWest(void);
    void RS232stopMoveNorth(void);
    void RS232stopMoveSouth(void);
    void RS232moveEast(void);
    void RS232moveWest(void);
    void RS232moveNorth(void);
    void RS232moveSouth(void);
    void RS232centerSpeed(void);
    void RS232guideSpeed(void);
    void RS232findSpeed(void);
    void RS232gotoSpeed(void);
};

#endif // LX200_COMMUNICATION_H
