#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include <QtConcurrent/qtconcurrentrun.h>
#include "qstepperphidgets.h"
#include "qencoderphidgets.h"
#include "alccd5_client.h"

using namespace QtConcurrent;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void emergencyShutdown(short);

private slots:
    void updateReadings(void);
    void executeSteps(void);
    void shutDownProgram(void);
    void setMaxStepperAcc(void);
    void setMaxStepperVel(void);
    void setINDISAddrAndPort(void);
    void takeSingleCamShot(void);

private:
    Ui::MainWindow *ui;
    QStepperPhidgets *StepperDriveOne;
    QEncoderPhidgets *EncoderDriveOne;
    QTimer *timer;
    QFuture<void> futureStepperBehaviour;
    alccd5_client *camera_client;
    QPixmap *camImg;
    void updateCameraImage(void);
};

#endif // MAINWINDOW_H
