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
    void declinationMoveHandboxUp(void);
    void declinationMoveHandboxDown(void);
    void declinationMoveHandboxStop(void);

private:
    Ui::MainWindow *ui;
    QStepperPhidgets *dummyDrive;
    QStepperPhidgets *StepperDriveRA;
    QStepperPhidgets *StepperDriveDecl;
    QTimer *timer;
    QFuture<void> futureStepperBehaviourRA;
    QFuture<void> futureStepperBehaviourDecl;
    alccd5_client *camera_client;
    QPixmap *camImg;
    currentObjectCatalog *objCatalog;
    void updateCameraImage(void);
    QDisplay2D *camView;
    float ra; // right ascension of a current object
    float decl;// declination of a current object
    bool RATrackingIsOn;
    qint64 RAtrackingElapsedTimeInMS; // timestamp for elapsed time of the tracking since last call to clock-sync

};

#endif // MAINWINDOW_H
