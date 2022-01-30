// this code is part of "TSC", a free control software for astronomical telescopes
// Copyright (C)  2016-19, wolfgang birkfellner
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

//---------------------------------------------------

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include <QListWidgetItem>
#include <QElapsedTimer>
#include <QFile>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QTimeZone>
#include <QProcess>
#include <stdlib.h>
#include "QtContinuousStepper.h"
#include "QtKineticStepper.h"
#include "ccd_client.h"
#include "currentObjectCatalog.h"
#include "QDisplay2D.h"
#include "lx200_communication.h"
#include "wiringPi.h"
#include "ocv_guiding.h"
#include "spi_drive.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    enum driveSpeed {guideTrack, move, slew};
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots: // callbacks for (mainly) GUI widgets
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
    void selectGuiderCamDriverName(void);
    void selectMainCamDriverName(void);
    void startCCDAcquisition(void);
    void stopCCDAcquisition(void);
    void storeMainCCDData(void);
    void selectCameraTypes(void);
    void syncMount(void);
    void syncMount(float, float, bool);
    void syncMountFromGoTo(void);
    void storeGearData(void);
    void storeDriveData(void);
    void catalogChosen(QListWidgetItem*);
    void catalogObjectChosen(void);
    void declinationMoveHandboxUp(void);
    void declinationMoveHandboxDown(void);
    void RAMoveHandboxFwd(void);
    void RAMoveHandboxBwd(void);
    void killHandBoxMotion(void);
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
    void displayGuideCamImage(QPixmap*);
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
    void handleHandbox(void);
    void readST4Port(void);
    void startST4Guiding(void);
    void stopST4Guiding(void);
    void storeSiteData(void);
    void setTrackingRate(void);
    void handleDSLRSingleExposure(void);
    void startDSLRSeries(void);
    void takeNextExposureInSeries(void);
    void stopDSLRExposureSeries(void);
    void terminateDSLRSeries(void);
    void transferCoordinates(void);
    void terminateDSLRSingleShot(void);
    void emergencyStopAuxDrives(void);
    void storeAuxBoardParams(void);
    void mvAux1FwdFull(void);
    void mvAux1BwdFull(void);
    void mvAux2FwdFull(void);
    void mvAux2BwdFull(void);
    void mvAux1FwdSmall(void);
    void mvAux1BwdSmall(void);
    void mvAux2FwdSmall(void);
    void mvAux2BwdSmall(void);
    void mvAux1FwdTiny(void);
    void mvAux1BwdTiny(void);
    void mvAux2FwdTiny(void);
    void mvAux2BwdTiny(void);
    void mvGuideAuxFwdFull(void);
    void mvGuideAuxBwdFull(void);
    void mvGuideAuxFwdSmall(void);
    void mvGuideAuxBwdSmall(void);
    void mvGuideAuxFwdTiny(void);
    void mvGuideAuxBwdTiny(void);
    void updateAuxDriveStatus(void);
    void storeDSLRSettingsForDithering(void);
    void terminateGuiderCalibration(void);
    void confirmGuideStar(void);
    void skipCalibration(void);
    void getTemperature(void);
    void IPaddressForHandboxChosen(void);
    void connectHandboxToIPSocket(void);
    void disconnectHandboxFromIPSocket(void);
    void establishHBIPLink(void);
    void sendDataToTCPHandboxSlot(void);
    void storeHandBoxSpeeds(void);
    void determineParkingPosition(void);
    void gotoParkPosition(void);
    void syncParkPosition(void);
    void handleSerialLXCB(void);
    void mountIsGerman(void);
    void mountIsEast(void);
    void setDecForNoFlip(void);
    void setTimeFromLX200Flag(void);
    void updateLocalization(void);
    void presetParkingPositionPolaris(void);
    void presetParkingPositionSouthH(void);
    void getDriveError(void);
    void storeSettingsForPlateSolving(void);
    void psTakeImage(void);
    void psChooseFITSDirectory(void);
    void psStartSolving(void);
    void psSetSearchRadiusForPS(void);
    void psHandleEndOfAstronomyNetProcess(int, QProcess::ExitStatus);
    void psKillAstrometryNet(void);
    void psDisplayAstrometryNetOutput(void);
    void psReadWCSInfoOutput(void);
    void syncPSCoordinates(void);
    void displayMainCamImage(QPixmap*);
    void mainCamPropertySelected(void);
    void guideCamPropertySelected(void);
    void mainCamSetINDINumberProperties(void);
    void guideCamSetINDINumberProperties(void);
    void mainCamSendINDINumber(void);
    void guideCamSendINDINumber(void);
    void mainCamSetINDITextProperties(void);
    void guideCamSetINDITextProperties(void);
    void mainCamSendINDIText(void);
    void guideCamSendINDIText(void);
    void mainCamSetINDISwitchProperties(void);
    void guideCamSetINDISwitchProperties(void);
    void mainCamSendINDISwitch(void);
    void guideCamSendINDISwitch(void);
    void loadPropertyList(void);
    void indicateNumberOnINDIServerMainCCD(void);
    void indicateTextOnINDIServerMainCCD(void);
    void indicateSwitchOnINDIServerMainCCD(void);
    void indicateNumberOnINDIServerGuiderCCD(void);
    void indicateTextOnINDIServerGuiderCCD(void);
    void indicateSwitchOnINDIServerGuiderCCD(void);
    void mainCamSetINDINumberOnServer(void);
    void guideCamSetINDINumberOnServer(void);
    void deployINDIMsgDlg(void);
    void saveMainCCDConfig(void);
    void saveGuideCCDConfig(void);
    void checkGPSFix(void);

private:
    struct mountMotionStruct { // a struct holding all relevant data ont the state of the mount
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
        bool btMoveNorth; // true when handbox command is active
        bool btMoveEast;
        bool btMoveSouth;
        bool btMoveWest;
    };
    struct currentGuideStarPosition { // used in autoguiding
        float centrX;
        float centrY;
    };

    struct guidingStateStruct { // holds all relevant information during autoguiding (not ST4)
        bool guideStarSelected; // true if a gudie star is selected
        bool guidingIsOn; // true if guiding us running
        bool calibrationIsRunning; // true during calibration of the autoguider
        bool systemIsCalibrated; // true if a pulse length and a rotation matrix is found
        bool calibrationImageReceived; // false while waiting for a camera image for autoguider calibration
        short declinationDriveDirection; // if a mirror/prism is involved, mirroring the autoguider is necessary
        double travelTime_ms_RA; // correction pulse in milliseconds in right ascension
        double travelTime_ms_Decl; // same for declination
        double rotationAngle; // relative angle between mount and camera coordinate system
        double maxDevInArcSec; // maximum error in seconds of arc duign guiding
        double rmsDevInArcSec; // rms error in " during guiding
        double rmsDevInArcSecSum; // running sum of squared errors for RMS computation
        double backlashCompensationInMS; // additional pulse to be issued if declination travel is inverted
        long noOfGuidingSteps; // number of acquired autoguider images
        bool st4IsActive; // true if ST4 is active
        float raErrs[3];
        float declErrs[3];
        long trackingSpeedRA;
    };

    struct DSLRStateStruct {
        QElapsedTimer dslrExpElapsed;
        bool dslrExposureIsRunning;
        bool dslrSeriesRunning;
        int dslrExpTime;
        int noOfExposures;
        int noOfExposuresLeft;
        double ditherTravelInMSRA;
        double ditherTravelInMSDecl;
        float tempAtSeriesStart;
    };

    struct currentCommunicationParameters {
        bool chan0IsOpen;
        bool chan1IsOpen;
        QString *guiData;
    };

    struct ST4StateStruct {
        QElapsedTimer *raCorrTime;
        QElapsedTimer *deCorrTime;
        bool nActive;
        bool eActive;
        bool sActive;
        bool wActive;
    };

    Ui::MainWindow *ui;
    struct mountMotionStruct mountMotion;
    struct currentGuideStarPosition guideStarPosition;
    struct guidingStateStruct guidingState;
    struct DSLRStateStruct dslrStates;
    struct currentCommunicationParameters commSPIParams;
    struct ST4StateStruct st4State;
    driveSpeed raState = guideTrack;
    driveSpeed deState = guideTrack;
    QtContinuousStepper *StepperDriveRA;
    QtKineticStepper *StepperDriveDecl;
    QTimer *timer;
    QTimer *st4Timer;
    QTimer *LX200Timer;
    QTimer *auxDriveUpdateTimer;
    QTimer *tempUpdateTimer;
    QTimer *tcpHandBoxSendTimer;
    QTimer *checkDriveTimer;
    QTimer *checkGPSFixTimer;
    QDate *UTDate;
    QTime *UTTime;
    QTimeZone *timeZone;
    double julianDay;
    lx200_communication *lx200Comm;
    QPixmap *camImg;
    QPixmap *mainCamImg;
    QPixmap *guideStarPrev;
    currentObjectCatalog *objCatalog;
    QDisplay2D *camView;
    QElapsedTimer *elapsedGoToTime;
    QElapsedTimer *elapsedPS;
    long psImageAcquisionTimeRemaining;
    ocv_guiding *guiding; // the class that does image processing for guiding
    ccd_client *camera_client;
    QString* guideCamDriverName;
    QString* mainCamDriverName;
    bool cam1Selected = false;
    bool cam2Selected = false;
    bool guiderCamSelected = false;
    bool mainCamSelected = false;
    bool camSelectionFinished = false;
    bool GPSHasFix = false;
    QTcpServer *LXServer;
    QTcpServer *HBServer;
    QTcpSocket *LXSocket;
    QTcpSocket *HBSocket;
    QHostAddress *LXServerAddress;
    QHostAddress *HBServerAddress;
    QByteArray *tcpLXdata;
    QByteArray *tcpHBData;
    QSerialPort *lx200SerialPort;
    QByteArray *lx200SerialData;
    SPI_Drive *spiDrOnChan1;
    SPI_Drive *spiDrOnChan0;
    short initiateStepperDrivers(void);
    void terminateGoTo(bool);
    bool LX200SerialPortIsUp;
    bool camImageWasReceived; // a flag set to true if a cam image came in
    bool lx200IsOn;
    bool ccdGuiderCameraIsAcquiring = false;
    bool ccdMainCameraIsAcquiring = false;
    bool auxBoardIsAvailable = 0;
    bool tcpHandboxIsConnected = 0;
    bool trackingBeforeHandboxMotionStarted = false; // a flag that indicates what was the RA motion before a handbox motion started
    bool auxDriveIsStartingUp = false; // a flag that suppresses GUI updates when one of the focus motors comes up
    bool calibrationToBeTerminated = 0; // a flag that is set when the calibration process is stopped prematurely
    bool isInParking = false; // set to true if parking was initiated
    bool meridianFlipDisabledForPolarParking = false; // when moving to the polar parking position, flipping is disabled
    float ra; // right ascension of a current object
    float decl;// declination of a current object
    double gotoETA; // estimated time of arrival for goto
    float targetRA;
    float targetDecl;  // coordinates for GoTo
    float psRA = 0;
    float psDecl = 0; // coordinates from platesolving
    short RAdriveDirectionForNorthernHemisphere;
    double approximateGOTOSpeedDecl;  // for display of travel, store an average travel speed here,
    double approximateGOTOSpeedRA;    // taking into account the acceleration ramps...
    float guidingFOVFactor;
    double rotMatrixGuidingXToRA[2][2];
    float temperature;
    int pulseGuideDuration;
    QString *textEntry;
    QString *bt_HandboxCommand;
    QString *currentRAString;
    QString *currentDeclString;
    QString *currentHAString;
    QString *coordString;
    QString *wcsInfoOutput;
    QFile *guidingLog;
    QProcess *astroMetryProcess;
    qint64 *ametryPID;
    bool isDriveActive(bool);
    void connectLX200Events(bool);
    void updateTimeAndDate(void);
    void declinationPulseGuide(long, short);
    void raPulseGuide(long, short);
    void emergencyShutdown(short);
    void setControlsForRATravel(bool);
    void setControlsForRATracking(bool);
    void setControlsForDeclTravel(bool);
    void setControlsForGoto(bool);
    void setControlsForGuiding(bool);
    void setControlsForAutoguiderCalibration(bool);
    void setAuxDriveControls(bool);
    void terminateAllMotion(void);
    void takeSingleGuiderCamShot(void);
    void takeSingleMainCamShot(void);
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
    bool getCCDParameters(bool);
    void setINDIrbuttons(bool);
    void shutDownPort(void);
    void openPort(void);
    void updateDSLRGUIAndCountdown(void);
    void carryOutDitheringStep(void);
    void waitForNMSecs(int);
    void checkDrivesForActivity(void);
    bool checkForController(void);
    void sendStepsToAuxController(short, bool, short);
    void sendAccToAuxController(void);
    void sendSpeedToAuxController(void);
    void sendMicrostepsToController(void);
    void enableAuxDrives(short, bool);
    void moveAuxDrive(short);
    void stopAuxDrive(short);
    void moveAuxPBSlot(short, bool, short);
    void moveGuiderAuxPBSlot(short, bool, short);
    void calibrationTerminationStuffToBeDone(void);
    int getMStepRatios(short);
    void mvAux1FwdFullHB(void);
    void mvAux1BwdFullHB(void);
    void mvAux2FwdFullHB(void);
    void mvAux2BwdFullHB(void);
    void mvAux1FwdSmallHB(void);
    void mvAux1BwdSmallHB(void);
    void mvAux2FwdSmallHB(void);
    void mvAux2BwdSmallHB(void);
    void mvAux1FwdTinyHB(void);
    void mvAux1BwdTinyHB(void);
    void mvAux2FwdTinyHB(void);
    void mvAux2BwdTinyHB(void);
    void readTCPHandboxData(void);
    void sendDataToTCPHandbox(QString);
    QString* generateCoordinateString(float, bool);
    short checkForFlip(bool, float, float, float, float);
    double psComputeFOVForMainCCD(void);
    void psreadCoordinatesFromFITS(void);
    void resetMainCamINDIPropertyGUIElements(void);
    void resetGuideCamINDIPropertyGUIElements(void);
    void getINDISwitchRules(bool);
    void storeGuiderCCDData(void);

signals:
    void dslrExposureDone(void);
    void tcpHandboxDataReceived(void);

};

#endif // MAINWINDOW_H
