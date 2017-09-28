#ifndef TSC_BT_SERIALCOMM_H
#define TSC_BT_SERIALCOMM_H

#include <QObject>
#include <QString>
#include <QCoreApplication>
#include <QSerialPort>
#include <QSerialPortInfo>

class tsc_bt_serialcomm :public QObject {
    Q_OBJECT
public:
    tsc_bt_serialcomm(QString);
    void bt_serialcommTryRestart(QString);
    ~tsc_bt_serialcomm(void);
    void shutDownPort(void);
    void openPort(void);
    bool getPortState(void);
    qint64 getDataFromSerialPort(void);
    QString* getTSCcommand(void);

private:
    QSerialPort rfcommport;
    QString *incomingCommand;
    bool portIsUp;

signals:
    void btDataReceived();
};
#endif
// TSC_BT_SERIALCOMM_H
