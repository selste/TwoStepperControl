#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "spi_drive.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void getAcc(void);
    void getVel(void);
    void getSteps(void);
    void enableDrive(short, short);
    void getMSteps(void);

private:
    Ui::MainWindow *ui;
    QTimer *timer;
    SPI_Drive *spiDrOnChan0;
    SPI_Drive *spiDrOnChan1;
    struct currentCommunicationParameters {
        bool chan0IsOpen;
        bool chan1IsOpen;
        int selectedChannel;
        QString *guiData;
    };
    currentCommunicationParameters commParams;
    void waitForNMSecs(int);

private slots:
    void startDrive(void);
    void stopDrive(void);
    void sendKinematics(void);
    void terminateProgram(void);
    void switchChannel(void);
    void checkDrivesForActivity(void);
    void checkForController(void);
};

#endif // MAINWINDOW_H
