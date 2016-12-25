#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include <QtConcurrent/qtconcurrentrun.h>
#include <QListWidgetItem>
#include <QElapsedTimer>
#include <stdlib.h>
#include "qstepperphidgetsRA.h"
#include "qstepperphidgetsDecl.h"
#include "alccd5_client.h"
#include "currentObjectCatalog.h"
#include "QDisplay2D.h"
#include "lx200_communication.h"

using namespace QtConcurrent;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

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
    void switchToLX200(void);
    void LXmoveEast(void);
    void LXmoveWest(void);
    void LXmoveNorth(void);
    void LXmoveSouth(void);
    void LXstopMoveEast(void);
    void LXstopMoveWest(void);
    void LXstopMoveNorth(void);
    void LXstopMoveSouth(void);
    void LXstopMotion(void);
    void LXslowSpeed(void);
    void LXhiSpeed(void);
    void LXsyncMount(void);
    void LXslewMount(void);
    void displayGuideCamImage(void);
    void emergencyStop(void);
    bool getCCDParameters(void);
    void storeCCDData(void);
    void handleServerMessage(void);
    void deployINDICommand(void);
    void declPGPlus(void);
    void declPGMinus(void);
    void raPGFwd(void);
    void raPGBwd(void);


private:
    struct mountMotionStruct {
        bool RATrackingIsOn;  // true when the telescope is in tracking mode
        bool RADriveIsMoving; // true when the RA drive moves but does not track
        bool DeclDriveIsMoving; // true when the Decl drive moves
        bool GoToIsActiveInRA; // the flag for hi-speed motion in RA
        bool GoToIsActiveInDecl; // the flag for hi-speed motion in decl
        bool emeregencyStopTriggered; // a flag that is set when the Emergency Stop button is pressed
        double DeclDriveDirection;
        double RADriveDirection;
        double RASpeedFactor;
        double DeclSpeedFactor;
        qint64 RAtrackingElapsedTimeInMS; // timestamp for elapsed time of the tracking since last call to clock-sync
        qint64 RAMoveElapsedTimeInMS;
        qint64 DeclMoveElapsedTimeInMS; // timestamp for elapsed time of the tracking since last call to clock-sync
        qint64 RAGoToElapsedTimeInMS;
        qint64 DeclGoToElapsedTimeInMS;
    };
    Ui::MainWindow *ui;
    struct mountMotionStruct mountMotion;
    QStepperPhidgetsRA *StepperDriveRA;
    QStepperPhidgetsDecl *StepperDriveDecl;
    QTimer *timer;
    lx200_communication *lx200port;
    bool lx200IsOn;
    bool MountWasSynced;     // a flag indicating whether a sync occurred
    QFuture<void> futureStepperBehaviourRATracking;
    QFuture<void> futureStepperBehaviourRA;
    QFuture<void> futureStepperBehaviourDecl;
    QFuture<void> futureStepperBehaviourRA_GOTO;
    QFuture<void> futureStepperBehaviourDecl_GOTO;
    QFuture<void> futureStepperBehaviourRA_Corr;
    QFuture<void> futureStepperBehaviourDecl_Corr;
    alccd5_client *camera_client;
    QPixmap *camImg;
    currentObjectCatalog *objCatalog;
    void updateCameraImage(void);
    void declinationPulseGuide(long, short);
    void raPulseGuide(long, short);
    void emergencyShutdown(short);
    void setControlsForRATravel(bool);
    void setControlsForRATracking(bool);
    void setControlsForDeclTravel(bool);
    void setControlsForGoto(bool);
    void terminateAllMotion(void);
    QDisplay2D *camView;
    float ra; // right ascension of a current object
    float decl;// declination of a current object
    double gotoETA; // estimated time of arrival for goto
    QElapsedTimer *elapsedGoToTime;
    short RAdriveDirectionForNorthernHemisphere;
    QString *textEntry;
    double approximateGOTOSpeedDecl;  // for display of travel, store an average travel speed here,
    double approximateGOTOSpeedRA;    // taking into account the acceleration ramps...
};

#endif // MAINWINDOW_H
