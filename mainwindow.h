#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include <QtConcurrent/qtconcurrentrun.h>
#include <QListWidgetItem>
#include <QElapsedTimer>
#include <QFile>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <stdlib.h>
#include "qstepperphidgetsRA.h"
#include "qstepperphidgetsDecl.h"
#include "ccd_client.h"
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
    void disconnectFromINDIServer(void);
    void clearINDILog(void);
    void findOutAboutINDIServerPID(void);
    void killRunningINDIServer(void);
    void startCCDAcquisition(void);
    void stopCCDAcquisition(void);
    void changeCCDGain(void);
    void storeCCDData(void);
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
    void IPaddressChosen(void);
    void connectToIPSocket(void);
    void disconnectFromIPSocket(void);
    void establishLX200IPLink(void);
    void handleRAviaTCP(QString*);
    void handleDeclviaTCP(QString*);
    void handleCommandviaTCP(QString*);
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
    void handleServerMessage(void);
    void deployINDICommand(void);
    void declPGPlus(void);
    void declPGMinus(void);
    void raPGFwd(void);
    void raPGBwd(void);
    void readLX200Port(void);
    void logLX200IncomingCmds(void);
    void logLX200OutgoingCmds(void);
    void logLX200OutgoingCmdsRA(void);
    void logLX200OutgoingCmdsDecl(void);
    void sendPolarAlignmentCommand(void);
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
    void resetGuidingCalibration(void);
    void resetGuidingError(void);
    void startBTComm(void);
    void stopBTComm(void);
    void restartBTComm(void);
    void handleBTHandbox(void);
    void readST4Port(void);
    void startST4Guiding(void);
    void stopST4Guiding(void);
    void storeSiteData(void);

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
        bool st4IsActive;
    };

    struct ST4stateDurationsStruct{
        bool declTimeMeasurementActive;
        bool RATimeMeasurementActive;
        long dpDuration;
        long dmDuration;
        long rpDuration;
        long rmDuration;
        QElapsedTimer dElapsed;
        QElapsedTimer rElapsed;
    };

    Ui::MainWindow *ui;
    struct mountMotionStruct mountMotion;
    struct currentGuideStarPosition guideStarPosition;
    struct guidingStateStruct guidingState;
    struct ST4stateDurationsStruct ST4stateDurations;
    QStepperPhidgetsRA *StepperDriveRA;
    QStepperPhidgetsDecl *StepperDriveDecl;
    QTimer *timer;
    QTimer *st4Timer;
    QTimer *LX200Timer;
    QDate *UTDate;
    QTime *UTTime;
    qint64 julianDay;
    lx200_communication *lx200port;
    bt_serialcomm *bt_Handbox;
    QPixmap *camImg;
    QPixmap *guideStarPrev;
    currentObjectCatalog *objCatalog;
    QDisplay2D *camView;
    QElapsedTimer *elapsedGoToTime;
    ocv_guiding *guiding; // the class that does image processing for guiding
    QFuture<void> futureStepperBehaviourRATracking;
    QFuture<void> futureStepperBehaviourRA;
    QFuture<void> futureStepperBehaviourDecl;
    QFuture<void> futureStepperBehaviourRA_GOTO;
    QFuture<void> futureStepperBehaviourDecl_GOTO;
    QFuture<void> futureStepperBehaviourRA_Corr;
    QFuture<void> futureStepperBehaviourDecl_Corr;
    ccd_client *camera_client;
    QTcpServer *LXServer;
    QTcpSocket *LXSocket;
    QHostAddress *LXServerAddress;
    QByteArray *tcpLXdata;
    QSerialPort *lx200SerialPort;
    QByteArray *lx200SerialData;
    bool LX200SerialPortIsUp;
    bool camImageWasReceived; // a flag set to true if a cam image came in
    bool lx200IsOn;
    bool ccdCameraIsAcquiring;
    float ra; // right ascension of a current object
    float decl;// declination of a current object
    double gotoETA; // estimated time of arrival for goto
    short RAdriveDirectionForNorthernHemisphere;
    double approximateGOTOSpeedDecl;  // for display of travel, store an average travel speed here,
    double approximateGOTOSpeedRA;    // taking into account the acceleration ramps...
    float guidingFOVFactor;
    double rotMatrixGuidingXToRA[2][2];
    QString *textEntry;
    QString *bt_HandboxCommand;
    QFile *guidingLog;
    void updateTimeAndDate(void);
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
    void waitForDriveStop(bool,bool);
    bool abortCCDAcquisition(void);
    void displayCalibrationStatus(QString, float, QString);
    void displayCalibrationStatus(QString);
    void declPGPlusGd(long);
    void declPGMinusGd(long);
    void raPGFwdGd(long);
    void raPGBwdGd(long);
    void compensateDeclBacklashPG(short);
    void handleST4State(void);
    void doDeclinationMoveForST4(short);
    bool getCCDParameters(void);
    void setINDIrbuttons(bool);
    void shutDownPort(void);
    void openPort(void);
};

#endif // MAINWINDOW_H
