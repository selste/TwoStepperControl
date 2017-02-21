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
#include "wiringPi.h"
#include "ocv_guiding.h"
#include "bt_serialcomm.h"

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
    void startCCDAcquisition(void);
    void stopCCDAcquisition(void);
    void changeCCDGain(void);
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
    void logLX200IncomingCmds(void);
    void logLX200OutgoingCmds(void);
    void logLX200OutgoingCmdsRA(void);
    void logLX200OutgoingCmdsDecl(void);
    void clearLXLog(void);
    void LXSetNumberFormatToSimple(void);
    void enableCamImageStorage(void);
    void selectGuideStar(void);
    void doAutoGuiding(void);
    void displayGuideStarPreview(void);
    void changePrevImgProc(void);
    void changeGuideScopeFL(void);
    void storeGuideScopeFL(void);
    void setHalfFOV(void);
    void setDoubleFOV(void);
    void setRegularFOV(void);
    void calibrateAutoGuider(void);
    void startBTComm(void);
    void stopBTComm(void);
    void restartBTComm(void);
    void handleBTHandbox(void);

private:
    struct mountMotionStruct {
        bool RATrackingIsOn;  // true when the telescope is in tracking mode
        bool RADriveIsMoving; // true when the RA drive moves but does not track
        bool DeclDriveIsMoving; // true when the Decl drive moves
        bool GoToIsActiveInRA; // the flag for hi-speed motion in RA
        bool GoToIsActiveInDecl; // the flag for hi-speed motion in decl
        bool emergencyStopTriggered; // a flag that is set when the Emergency Stop button is pressed
        double DeclDriveDirection;
        double RADriveDirection;
        double RASpeedFactor;
        double DeclSpeedFactor;
        qint64 RAtrackingElapsedTimeInMS; // timestamp for elapsed time of the tracking since last call to clock-sync
        qint64 RAMoveElapsedTimeInMS;
        qint64 DeclMoveElapsedTimeInMS; // timestamp for elapsed time of the tracking since last call to clock-sync
        qint64 RAGoToElapsedTimeInMS;
        qint64 DeclGoToElapsedTimeInMS;
        bool btMoveNorth;
        bool btMoveEast;
        bool btMoveSouth;
        bool btMoveWest;
    };
    struct currentGuideStarPosition {
        float centrX;
        float centrY;
    };

    struct guidingStateStruct {
        bool guideStarSelected;
        bool guidingIsOn;
        bool calibrationIsRunning;
        bool systemIsCalibrated;
        bool calibrationImageReceived;
        short declinationDriveDirection;
        double travelTime_ms;
        double rotationAngle;
        double maxDevInArcSec;
        double backlashCompensationInMS;
        long noOfGuidingSteps;
    };

    Ui::MainWindow *ui;
    struct mountMotionStruct mountMotion;
    struct currentGuideStarPosition guideStarPosition;
    struct guidingStateStruct guidingState;
    QStepperPhidgetsRA *StepperDriveRA;
    QStepperPhidgetsDecl *StepperDriveDecl;
    QTimer *timer;
    lx200_communication *lx200port;
    bt_serialcomm *bt_Handbox;
    bool camImageWasReceived; // a flag set to true if a cam image came in
    bool lx200IsOn;
    bool MountWasSynced;     // a flag indicating whether a sync occurred
    bool ccdCameraIsAcquiring;
    QFuture<void> futureStepperBehaviourRATracking;
    QFuture<void> futureStepperBehaviourRA;
    QFuture<void> futureStepperBehaviourDecl;
    QFuture<void> futureStepperBehaviourRA_GOTO;
    QFuture<void> futureStepperBehaviourDecl_GOTO;
    QFuture<void> futureStepperBehaviourRA_Corr;
    QFuture<void> futureStepperBehaviourDecl_Corr;
    alccd5_client *camera_client;
    QPixmap *camImg;
    QPixmap *guideStarPrev;
    currentObjectCatalog *objCatalog;
    void updateCameraImage(void);
    void declinationPulseGuide(long, short,bool);
    void raPulseGuide(long, short,bool);
    void emergencyShutdown(short);
    void setControlsForRATravel(bool);
    void setControlsForRATracking(bool);
    void setControlsForDeclTravel(bool);
    void setControlsForGoto(bool);
    void setControlsForGuiding(bool);
    void setControlsForAutoguiderCalibration(bool);
    void terminateAllMotion(void);
    void takeSingleCamShot(void);
    double correctGuideStarPosition(float, float);
    void waitForCalibrationImage(void);
    bool abortCCDAcquisition(void);
    void displayCalibrationStatus(QString, float, QString);
    void displayCalibrationStatus(QString);
    QDisplay2D *camView;
    float ra; // right ascension of a current object
    float decl;// declination of a current object
    double gotoETA; // estimated time of arrival for goto
    QElapsedTimer *elapsedGoToTime;
    short RAdriveDirectionForNorthernHemisphere;
    QString *textEntry;
    QString *bt_HandboxCommand;
    double approximateGOTOSpeedDecl;  // for display of travel, store an average travel speed here,
    double approximateGOTOSpeedRA;    // taking into account the acceleration ramps...
    ocv_guiding *guiding; // the class that does image processing for guiding
    float guidingFOVFactor;
    double rotMatrixGuidingXToRA[2][2];
    void declPGPlusGd(long);
    void declPGMinusGd(long);
    void raPGFwdGd(long);
    void raPGBwdGd(long);
    void compensateDeclBacklashPG(short);
};

#endif // MAINWINDOW_H
