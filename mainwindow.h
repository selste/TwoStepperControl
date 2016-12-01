#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include <QtConcurrent/qtconcurrentrun.h>
#include <QListWidgetItem>
#include <QElapsedTimer>
#include "qstepperphidgetsRA.h"
#include "qstepperphidgetsDecl.h"
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
    void setControlsForRATravel(bool);
    void setControlsForDeclTravel(bool);
    void setControlsForGoto(bool);

private slots:
    void updateReadings(void);
    void startRATracking(void);
    void stopRATracking(void);
    void shutDownProgram(void);
    void setMaxStepperAccRA(void);
    void setMaxStepperAccDecl(void);
    void setMaxStepperCurrentRA(void);
    void setMaxStepperCurrentDecl(void);
    void setINDISAddrAndPort(void);
    void takeSingleCamShot(void);
    void syncMount(void);
    void storeGearData(void);
    void storeDriveData(void);
    void catalogChosen(QListWidgetItem*);
    void catalogObjectChosen(void);
    void declinationMoveHandboxUp(void);
    void declinationMoveHandboxDown(void);
    void RAMoveHandboxFwd(void);
    void RAMoveHandboxBwd(void);
    void setCorrectionSpeed(void);
    void setMoveSpeed(void);
    void startGoToObject(void);
    void changeMoveSpeed(void);
    void invertRADirection(void);

private:
    struct mountMotionStruct {
        bool RATrackingIsOn;  // true when the telescope is in tracking mode
        bool RADriveIsMoving; // true when the RA drive moves but does not track
        bool DeclDriveIsMoving; // true when the Decl drive moves
        bool GoToIsActiveInRA; // the flag for hi-speed motion in RA
        bool GoToIsActiveInDecl; // the flag for hi-speed motion in decl
        double DeclDriveDirection;
        double RADriveDirection;
        double RASpeedFactor;
        double DeclSpeedFactor;
        qint64 RAtrackingElapsedTimeInMS; // timestamp for elapsed time of the tracking since last call to clock-sync
        qint64 RAMoveElapsedTimeInMS;
        qint64 DeclMoveElapsedTimeInMS; // timestamp for elapsed time of the tracking since last call to clock-sync
    };
    Ui::MainWindow *ui;
    struct mountMotionStruct mountMotion;
    QStepperPhidgetsRA *StepperDriveRA;
    QStepperPhidgetsDecl *StepperDriveDecl;
    QTimer *timer;
    QFuture<void> futureStepperBehaviourRATracking;
    QFuture<void> futureStepperBehaviourRA;
    QFuture<void> futureStepperBehaviourDecl;
    QFuture<void> futureStepperBehaviourRA_GOTO;
    QFuture<void> futureStepperBehaviourDecl_GOTO;
    alccd5_client *camera_client;
    QPixmap *camImg;
    currentObjectCatalog *objCatalog;
    void updateCameraImage(void);
    QDisplay2D *camView;
    float ra; // right ascension of a current object
    float decl;// declination of a current object
    double gotoETA; // estimated time of arrival for goto
    QElapsedTimer *elapsedGoToTime;
    short RAdriveDirectionForNorthernHemisphere;
};

#endif // MAINWINDOW_H
