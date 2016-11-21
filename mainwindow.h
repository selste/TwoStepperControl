#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include <QtConcurrent/qtconcurrentrun.h>
#include <QListWidgetItem>
#include "qstepperphidgets.h"
#include "alccd5_client.h"
#include "currentObjectCatalog.h"
#include "QDisplay2D.h"

using namespace QtConcurrent;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void emergencyShutdown(short);

private slots:
    void updateReadings(void);
    void startRATracking(void);
    void stopRATracking(void);
    void shutDownProgram(void);
    void setMaxStepperAcc(void);
    void setMaxStepperVel(void);
    void setINDISAddrAndPort(void);
    void takeSingleCamShot(void);
    void syncMount(void);
    void storeGearData(void);
    void catalogChosen(QListWidgetItem*);
    void catalogObjectChosen(QListWidgetItem*);

private:
    Ui::MainWindow *ui;
    QStepperPhidgets *StepperDriveRA;
    QTimer *timer;
    QFuture<void> futureStepperBehaviour;
    alccd5_client *camera_client;
    QPixmap *camImg;
    currentObjectCatalog *objCatalog;
    void updateCameraImage(void);
    QDisplay2D *camView;
    bool trackingIsOn;
    float ra; // right ascension of a current object
    float decl;// declination of a current object
};

#endif // MAINWINDOW_H
