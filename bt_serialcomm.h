#ifndef BT_SERIALCOMM_H
#define BT_SERIALCOMM_H

#include <QObject>
#include <QString>
#include <QCoreApplication>
#include <QSerialPort>
#include <QSerialPortInfo>

class bt_serialcomm :public QObject {
    Q_OBJECT
public:
    bt_serialcomm(void);
    ~bt_serialcomm(void);
    void shutDownPort(void);
    void openPort(void);
    bool getPortState(void);
    qint64 getDataFromSerialPort(void);
    QString* getTSCcommand(void);

private:
    QSerialPort rfcommport;
    QString *incomingCommand;
    bool portIsUp;
};
#endif
// BT_SERIALCOMM_H
