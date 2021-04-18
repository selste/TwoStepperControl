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
//-------------------------------------------------------------------------------------
// this is the main class of TSC. it virtually manages all user interaction and all
// device - driven actions. these are timing tasks, management of the GUI and
// operations such as goto, guiding, reacting to user inputs and handboxes and
// interaction with external programs via ST4 and LX200.
// w. birkfellner, 2017-20

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qtimer.h>
#include <QDir>
#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QHostAddress>
#include <QtNetwork/QNetworkInterface>
#include <QTime>
#include <QTimeZone>
#include <QDate>
#include <QFileDialog>
#include <QCursor>
#include <math.h>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <QMessageBox>
#include <fitsio.h>
#include "QDisplay2D.h"
#include "tsc_globaldata.h"
#include "usb_communications.h"

TSC_GlobalData *g_AllData; // a global class that holds system specific parameters on drive, current mount position, gears and so on ...
usbCommunications *amisInterface;

//------------------------------------------------------------------
// constructor of the GUI - takes care of everything....
MainWindow::MainWindow(QWidget *parent):QMainWindow(parent), ui(new Ui::MainWindow) {
    double val; //local values for setting up stuff
    QDir *catalogDir; // the directory holding the local .tsc catalogs
    QFileInfoList catFiles; // a list of available .tsc files
    QFileInfo catFileInfo; // a helper on the file list of catalogs
    QStringList filter; // needed for isolating the .tsc files
    QString *catfName; // name of a .tsc catalog
    QString *currentYear; // sets the actual equinox
    QString *helper;
    QList<QHostAddress> ipAddressList;
    int isEastFlagFromFile;
    int listIter;
    bool foundDefaultIPForLX200 = false;
    bool foundDefaultIPForHBox = false;
    short auxMicrostepDenom, guiderFocusDrive;
    int msRat;
    QMessageBox timeZoneWrongMsg;

    ui->setupUi(this); // making the widget
    g_AllData =new TSC_GlobalData(); // instantiate the global class with parameters
    this->timeZone = new QTimeZone(QTimeZone::systemTimeZone()); // check whether the timezone is UTC
    if (this->timeZone->offsetFromUtc(QDateTime::currentDateTime()) != 0) {
        qDebug() << "TimeZoneOffset: " << this->timeZone->offsetFromUtc(QDateTime::currentDateTime());
        timeZoneWrongMsg.setWindowTitle("TSC critical timezone error");
        timeZoneWrongMsg.setText("Timezone is not UTC - I am trying to fix this with the data available. Check system time in the 'Location'-tab.");
        timeZoneWrongMsg.exec();
        system("sudo timedatectl set-timezone UTC");
        usleep(2000);
        system("sudo hwclock -w");
        usleep(250);
    }

    this->timer = new QTimer(); // start the event timer ... this is NOT the microtimer for the mount
    this->timer->start(100); // check all 100 ms for events
    elapsedGoToTime = new QElapsedTimer(); // timer for roughly measuring time taked during GoTo
    elapsedPS = new QElapsedTimer(); // timer for measuring time during image acquisition for platesolving
    this->st4Timer = new QTimer();
    this->LX200Timer = new QTimer();
    this->LX200Timer->start(150);
    this->auxDriveUpdateTimer = new QTimer();
    this->auxDriveUpdateTimer->start(500);
    this->tempUpdateTimer = new QTimer();
    this->tempUpdateTimer->start(30000);
    this->tcpHandBoxSendTimer = new QTimer();
    this->tcpHandBoxSendTimer->start(2000);
    this->checkDriveTimer = new QTimer();
    this->checkDriveTimer->start(5000);
    this->UTDate = new QDate(QDate::currentDate());
    this->julianDay = this->UTDate->toJulianDay();
    this->UTTime = new QTime(QTime::currentTime());
    this->UTTime->start();
    //long psImageAcquisionTimeRemaining = 0;
    this->findOutAboutINDIServerPID(); // check running processes and store the ID of an INDIserver in a hidden file
    currentYear = new QString(this->UTDate->currentDate().toString("yyyy"));
    ui->sbEpoch->setValue(currentYear->toInt());
    delete currentYear;
    this->wcsInfoOutput = new QString();

    this->initiateStepperDrivers(); // initialise the driver boards
    qDebug() << "Steppers initialized";

        // set a bunch of flags and factors
    this->mountMotion.RATrackingIsOn=false;   // sidereal tracking is on if true
    this->mountMotion.RADriveIsMoving =false; // RA drive is moving faster than sideral tracking if true
    this->mountMotion.DeclDriveIsMoving = false; // Decl drive is moving if true
    this->mountMotion.GoToIsActiveInRA = false; // system is in a slew state, RA is moving. most system functionality is disabled
    this->mountMotion.GoToIsActiveInDecl = false; // system is in a slew state, Decl is moving. most system functionality is disabled
    this->mountMotion.emergencyStopTriggered = false; // system can be halted by brute force. true if this was triggered
    this->lx200IsOn = false; // true if a serial connection was opened vai RS232
    this->ccdGuiderCameraIsAcquiring=false; // true if images are coming in from INDI-server
    this->ccdMainCameraIsAcquiring=false;
    if (ui->cbIsGEM->isChecked() == true) {
        ui->cbMountIsEast->setEnabled(true);
        g_AllData->switchDeclinationSign();
    }
    this->mountMotion.DeclDriveDirection = g_AllData->getMFlipDecSign(); // 1 for forward, -1 for backward
    this->mountMotion.RADriveDirection = 1; // 1 for forward, -1 for backward
    this->mountMotion.RASpeedFactor=1;
    this->mountMotion.DeclSpeedFactor=1; // speeds are multiples of sidereal compensation
    ui->rbCorrSpeed->setChecked(true); // activate radiobutton for correction speed ... this is sidereal speed
    this->guidingState.guideStarSelected=false;
    this->guidingState.guidingIsOn=false;

    g_AllData->setGuidingState(this->guidingState.guidingIsOn); // this has to be known in other classes, so every "guidingIsOn" state is copied
    this->guidingState.calibrationIsRunning=false;
    this->guidingState.systemIsCalibrated=false;
    this->guidingState.calibrationImageReceived=false;
    this->guidingState.travelTime_ms_RA = 50;
    this->guidingState.travelTime_ms_Decl = 50;
    this->guidingState.declinationDriveDirection=+1; // for backlash compensation - remember direction of last declination travel during guiding
    this->guidingState.rotationAngle=0.0;
    this->guidingState.maxDevInArcSec=0.0;
    this->guidingState.rmsDevInArcSec=0.0;
    this->guidingState.rmsDevInArcSecSum = 0.0;
    this->guidingState.backlashCompensationInMS = 0.0;
    this->guidingState.noOfGuidingSteps = 0;
    this->guidingState.st4IsActive = false;
    this->guidingState.raErrs[0] = this->guidingState.raErrs[1] = this->guidingState.raErrs[2] = 0;
    this->guidingState.declErrs[0] = this->guidingState.declErrs[1] = this->guidingState.declErrs[2] = 0;
    this->pulseGuideDuration = 500;
    this->dslrStates.dslrExposureIsRunning = false;
    this->dslrStates.dslrSeriesRunning = false;
    this->dslrStates.ditherTravelInMSRA = 0;
    this->dslrStates.ditherTravelInMSDecl = 0;
    ui->rbSiderealSpeed->setChecked(true); // make sure that sidereal speed is set...
//    this->setTrackingRate();
    ui->sbGoToSpeed->setValue(g_AllData->getHandBoxSpeeds(0));
    ui->sbMoveSpeed->setValue(g_AllData->getHandBoxSpeeds(1));
    ui->rbCorrSpeed->setChecked(true); // make sure that the loaded motion speed from prefs is set, but set the system to move speed ...

    this->guideCamDriverName = new QString();
    this->mainCamDriverName = new QString();
        // GPIO pins for DSLR control
    qDebug() << "Opening GPIO pins";
    setenv("WIRINGPI_GPIOMEM", "1", 1); // otherwise, the program needs sudo - privileges
    wiringPiSetup();
    pinMode (1, OUTPUT);
    pinMode (27, OUTPUT); // setting BCM-pins 18 and 16 to output mode for dslr-control
        // now setting all the parameters in the "Drive"-tab. settings are from pref-file, except for the stepper speed, which is
        // calculated from  gear parameters
    system("sudo timedatectl set-ntp 0");
    g_AllData->setDriveParams(0,0,this->StepperDriveRA->getKineticsFromController(3)); // velocity limit - this is set to sidereal speed for right ascension in the constructor of the stepper class ...
    g_AllData->setDriveParams(1,0,this->StepperDriveDecl->getKineticsFromController(3)); // velocity limit - this is set to sidereal speed for declination in the constructor of the stepper class ...
    this->StepperDriveRA->setStepperParams((g_AllData->getDriveParams(0,1)),1); // acceleration in RA
    this->StepperDriveRA->setStepperParams((g_AllData->getDriveParams(0,2)),3); // motor current in RA
    this->StepperDriveDecl->setStepperParams((g_AllData->getDriveParams(1,1)),1); // acceleration in Decl
    this->StepperDriveDecl->setStepperParams((g_AllData->getDriveParams(1,2)),3); // motor current in Decl
    val=(this->StepperDriveRA->getKineticsFromController(3));
    ui->lcdVMaxRA->display(round(val));
    val=(this->StepperDriveDecl->getKineticsFromController(3));
    ui->lcdVMaxDecl->display(round(val));
    ui->sbAMaxRA_AMIS->setValue((this->StepperDriveRA->getKineticsFromController(2)));
    ui->sbCurrMaxRA_AMIS->setValue((this->StepperDriveRA->getKineticsFromController(1)));
    ui->sbAMaxDecl_AMIS->setValue((this->StepperDriveDecl->getKineticsFromController(2)));
    ui->sbCurrMaxDecl_AMIS->setValue((this->StepperDriveDecl->getKineticsFromController(1)));
    ui->driverSelectWidget->setCurrentWidget(ui->tabAMIS);
    textEntry = new QString();
    ui->leRAPlanetary->setText(textEntry->number(g_AllData->getGearData(0)));
    textEntry->clear();
    ui->leRAGear->setText(textEntry->number(g_AllData->getGearData(1)));
    textEntry->clear();
    ui->leRAWorm->setText(textEntry->number(g_AllData->getGearData(2 )));
    textEntry->clear();
    ui->leRAStepsize->setText(textEntry->number(g_AllData->getGearData(3 )));
    textEntry->clear();
    ui->leDeclPlanetary->setText(textEntry->number(g_AllData->getGearData(4 )));
    textEntry->clear();
    ui->leDeclGear->setText(textEntry->number(g_AllData->getGearData(5 )));
    textEntry->clear();
    ui->leDeclWorm->setText(textEntry->number(g_AllData->getGearData(6 )));
    textEntry->clear();
    ui->leDeclStepSize->setText(textEntry->number(g_AllData->getGearData(7 )));
    textEntry->clear();
    qDebug() << "Drive params set ...";

        // now setting parameters in the "Settings"-tab
    ui->leControllerName->setText(g_AllData->getSiteName());
    ui->leLat->setText(textEntry->number(g_AllData->getSiteCoords(0)));
    textEntry->clear();
    ui->leLong->setText(textEntry->number(g_AllData->getSiteCoords(1)));
    textEntry->clear();
    ui->sbUTCOffs->setValue(g_AllData->getSiteCoords(2));
    ui->lcdHAPark->display(g_AllData->getParkingPosition(0));
    ui->lcdDecPark->display(g_AllData->getParkingPosition(1));
    const QFileInfo outputDir((g_AllData->getPathToImages()).toLatin1()) ;
    if (outputDir.exists() && outputDir.isDir() && outputDir.isReadable() && outputDir.isWritable()) {
        ui->lePathToFitsFile->setText(g_AllData->getPathToImages());
    }
    ui->sbPSSearchRad->setValue(g_AllData->getPSSearchRad());
    this->astroMetryProcess = new QProcess();
    this->astroMetryProcess->setEnvironment( QProcess::systemEnvironment() );
    this->astroMetryProcess->setProcessChannelMode( QProcess::MergedChannels );
    ui->sbNoFlip->setValue(g_AllData->getMaxDeclForNoFlip());
    ui->cbIsGEM->setChecked(g_AllData->getMFlipParams(0));
    ui->cbMountIsEast->setChecked(g_AllData->getMFlipParams(1));
    if (ui->cbIsGEM->isChecked() == true) {
        ui->cbMountIsEast->setEnabled(true);
    }
    ui->cbTimeFromLX200->setChecked(g_AllData->getTimeFromLX200Flag());
    msRat = g_AllData->getMicroSteppingRatio(0);
    switch (msRat) {
        case 4: ui->rbNormal_4_AMIS->setChecked(true); break;
        case 8: ui->rbNormal_8_AMIS->setChecked(true); break;
        case 16: ui->rbNormal_16_AMIS->setChecked(true); break;
        case 32: ui->rbNormal_32_AMIS->setChecked(true); break;
        case 64: ui->rbNormal_64_AMIS->setChecked(true); break;
        case 128: ui->rbNormal_128_AMIS->setChecked(true); break;
        case 256: ui->rbNormal_256_AMIS->setChecked(true); break;
    }
    msRat = g_AllData->getMicroSteppingRatio(1);
    switch (msRat) {
        case 4: ui->rbMove_4_AMIS->setChecked(true); break;
        case 8: ui->rbMove_8_AMIS->setChecked(true); break;
        case 16: ui->rbMove_16_AMIS->setChecked(true); break;
        case 32: ui->rbMove_32_AMIS->setChecked(true); break;
        case 64: ui->rbMove_64_AMIS->setChecked(true); break;
        case 128: ui->rbMove_128_AMIS->setChecked(true); break;
        case 256: ui->rbMove_256_AMIS->setChecked(true); break;
    }
    msRat = g_AllData->getMicroSteppingRatio(2);
    switch (msRat) {
        case 4: ui->rbGoTo_4_AMIS->setChecked(true); break;
        case 8: ui->rbGoTo_8_AMIS->setChecked(true); break;
        case 16: ui->rbGoTo_16_AMIS->setChecked(true); break;
        case 32: ui->rbGoTo_32_AMIS->setChecked(true); break;
        case 64: ui->rbGoTo_64_AMIS->setChecked(true); break;
        case 128: ui->rbGoTo_128_AMIS->setChecked(true); break;
        case 256: ui->rbGoTo_256_AMIS->setChecked(true); break;
    } // setting up the buttons for the stored microstep ratios
        // camera and guiding class are instantiated
    camera_client = new ccd_client(); // install a camera client for guiding via INDI
    guiding = new ocv_guiding();
    guideStarPrev = new QPixmap(); // a pixmap for showing the preview window
    guideStarPosition.centrX =0.0;
    guideStarPosition.centrY =0.0;
    this->guidingFOVFactor=1.0; // location of the crosshair and size of the preview window are set
    ui->sbFLGuideScope->setValue(g_AllData->getGuideScopeFocalLength()); // get stored focal length for the guidescope
    this->guiding->setFocalLengthOfGuidescope(g_AllData->getGuideScopeFocalLength());
    this->guidingLog=NULL;
        // now read all catalog files, ending in "*.tsc"
    catalogDir = new QDir("Catalogs/");
    filter << "*.tsc";
    catalogDir->setNameFilters(filter);
    catFiles = catalogDir->entryInfoList();
    foreach (catFileInfo, catFiles) {
        catfName = new QString((const QString)catFileInfo.fileName());
        catfName->remove(((catfName->length())-4),4);
        ui->listWidgetCatalog->addItem(catfName->toLatin1());
        delete catfName;
    }
    delete catalogDir;
        // filled the selection with all ".tsc" files in the home directory
    this->objCatalog=NULL; // the topical catalogue
    this->ra = 0.0;
    this->decl = 0.0; // the sync position - no sync for the mount was carried out - these are displayed in the GOTO textentry
    this->camView = new QDisplay2D(ui->guidingTab,550,400); // make the clickable scene view of 425 x 340 pixels
    this->camImg = new QPixmap(g_AllData->getCameraDisplaySize(0,false),g_AllData->getCameraDisplaySize(1,false)); // store the size of the scene view in the global parameter class
    this->mainCamImg = new QPixmap(g_AllData->getCameraDisplaySize(0,true),g_AllData->getCameraDisplaySize(1,true));
    this->camImageWasReceived=false; // no camera image came in yet ...
    RAdriveDirectionForNorthernHemisphere = 1; //switch this for the southern hemisphere to -1 ... RA is inverted
    g_AllData->storeGlobalData();
    g_AllData->setSyncPosition(0.0, 0.0); // deploy a fake sync to the mount so that the microtimer starts ...
    this->LXServer = new QTcpServer();
    this->LXSocket = new QTcpSocket(this);
    this->LXServerAddress = new QHostAddress(); // creating a server, a socket and a hostaddress for the LX 200 tcp/ip server
    this->tcpLXdata = new QByteArray(); // a byte array holding the data coming in from the TCP/IP socket

    // repeat the above procedure for the TCP/IP handbox
    this->HBServer = new QTcpServer();
    this->HBSocket = new QTcpSocket(this);
    this->HBServerAddress = new QHostAddress(); // creating a server, a socket and a hostaddress for the LX 200 tcp/ip server
    this->tcpHBData = new QByteArray();
    this->tcpHandboxIsConnected=false;
    // read all available IP addresses and make them available for LX200
    ipAddressList = QNetworkInterface::allAddresses();
    for(listIter = 0; listIter < ipAddressList.count(); listIter++) {
        if ((ipAddressList[listIter].isLoopback() == false) && (ipAddressList[listIter].protocol() == QAbstractSocket::IPv4Protocol)) {
            ui->listWidgetIPAddresses->addItem(ipAddressList[listIter].toString());
            ui->listWidgetIPAddresses_2->addItem(ipAddressList[listIter].toString());
            if (ipAddressList[listIter].toString() == g_AllData->getLX200IPAddress()) {
                foundDefaultIPForLX200 = true;
            }
            if (ipAddressList[listIter].toString() == g_AllData->getHandboxIPAddress()) {
                foundDefaultIPForHBox = true;
            }
        }
    }

    // instantiate the class for serial communication via LX200
    this->LX200SerialPortIsUp = false;
    this->lx200SerialPort = new QSerialPort();
    this->lx200SerialPort->setPortName("/dev/ttyS0");
    this->lx200SerialPort->setBaudRate(QSerialPort::Baud9600);
    this->lx200SerialPort->setDataBits(QSerialPort::Data8);
    this->lx200SerialPort->setParity(QSerialPort::NoParity);
    this->lx200SerialPort->setStopBits(QSerialPort::OneStop);
    this->lx200SerialPort->setFlowControl(QSerialPort::NoFlowControl);
    this->lx200SerialData = new QByteArray();
    this->lx200Comm= new lx200_communication();
    this->LXSetNumberFormatToSimple(); // LX200 knows a simple and a complex number format for RA and Decl - set format to simple here ...
    // check whether LXserial is to be used by default
    ui->cbSerialLX200Default->setChecked(g_AllData->getLX200SerialFlag());

    // start either the serial port or try to start a TCP/IP server if a known address was found
    if (g_AllData->getLX200SerialFlag() == false) {
        helper = new QString("127.0.0.1");
        if (foundDefaultIPForLX200 == true) {
            ui->pbEnableTCP->setEnabled(true);
            ui->leDefaultIPAddressLX200->setText(g_AllData->getLX200IPAddress()->toLatin1());
            this->connectToIPSocket();
        } else {
            g_AllData->setLX200IPAddress(*helper);
            ui->leDefaultIPAddressLX200->setText("-");
        }
        delete helper;
    } else {
        switchToLX200();
    }

    // start the server for the handbox if a known address is available
    helper = new QString("127.0.0.1");
    if (foundDefaultIPForHBox == true) {
        ui->pbTCPHBEnable->setEnabled(true);
        ui->leDefaultIPAddressHBox->setText(g_AllData->getHandboxIPAddress()->toLatin1());
        this->connectHandboxToIPSocket();
    } else {
        g_AllData->setHandboxIPAddress(*helper);
        ui->leDefaultIPAddressHBox->setText("-");
    }
        // instantiate communications with handbox
    this->mountMotion.btMoveNorth=0;
    this->mountMotion.btMoveEast=0;
    this->mountMotion.btMoveSouth=0;
    this->mountMotion.btMoveWest=0;

      // now open SPI channel 0 mfor communication with the arduino on the head - this is the ADC for ST4 and temperature
    this->commSPIParams.guiData = new QString();
    this->spiDrOnChan0 = new SPI_Drive(0);
    if (this->spiDrOnChan0->spidrGetFD() != -1) {
        this->commSPIParams.chan0IsOpen = true;
    } else {
       this->commSPIParams.chan0IsOpen = false;
    }
        // start SPI communication via SPI channel #1 for auxiliary drives
    this->spiDrOnChan1 = new SPI_Drive(1);
    if (this->spiDrOnChan1->spidrGetFD() != -1) {
        this->commSPIParams.chan1IsOpen = true;
        ui->cbSPI1Open->setChecked(true);
    } else {
       this->commSPIParams.chan1IsOpen = false;
    }
    this->auxBoardIsAvailable = this->checkForController();
    if (this->auxBoardIsAvailable == true) {
        ui->gbFocuserInGuide->setEnabled(true);
        ui->fauxDrives->setEnabled(true);
        ui->leNameAux1->setText(g_AllData->getAuxName(0).toLatin1());
        ui->leNameAux2->setText(g_AllData->getAuxName(1).toLatin1());
        ui->sbStepsAux1->setValue(g_AllData->getStepsToBeDone(0));
        this->sendStepsToAuxController(0,false,0);
        ui->sbStepsAux2->setValue(g_AllData->getStepsToBeDone(1));
        this->sendStepsToAuxController(1,false,0);
        ui->sbAuxAcc->setValue(g_AllData->getAuxAcc());
        this->sendAccToAuxController();
        ui->sbAuxSpeed->setValue(g_AllData->getAuxSpeed());
        this->sendSpeedToAuxController();
        auxMicrostepDenom=g_AllData->getAuxMSteps();
        switch (auxMicrostepDenom) {
            case 1: ui->rbAuxMs1->setChecked(true); break;
            case 2: ui->rbAuxMs2->setChecked(true); break;
            case 4: ui->rbAuxMs4->setChecked(true); break;
            case 8: ui->rbAuxMs8->setChecked(true); break;
            case 16: ui->rbAuxMs16->setChecked(true); break;
            case 32: ui->rbAuxMs32->setChecked(true); break;
        }
        guiderFocusDrive = g_AllData->getGuiderFocusDrive();
        switch (guiderFocusDrive) {
            case 0: ui->rbNoFinderFocuser->setChecked(true); break;
            case 1: ui->rbNo1FinderFocuser->setChecked(true); break;
            case 2: ui->rbNo2FinderFocuser->setChecked(true); break;
        }
        this->sendMicrostepsToController();
    }
    this->waitForNMSecs(500); // give the microcontroller some time to breathe
    // ST 4 code
    if (this->commSPIParams.chan0IsOpen == true) {
        ui->pbStartST4->setEnabled(true);
        ui->cbSTFourIsWorking->setChecked(true);
    } else {
        ui->cbSTFourIsWorking->setChecked(false);
    } // ST4 signals are measured by the HAT Arduino and conveyed vis SPI channel 0; so if this channel is open, ST4 is available
    this->st4State.nActive = false; // a flag that is true when the North-line is closed
    this->st4State.eActive = false; // same as above for east
    this->st4State.sActive = false; // same as above for south
    this->st4State.wActive = false; // same as above for west
    this->st4State.raCorrTime = new QElapsedTimer();
    this->st4State.deCorrTime = new QElapsedTimer();
        // set the values for diagonal pixel size and main scope focal length in the DSLR settings
    ui->sbDSLRPixSize->setValue((double)(g_AllData->getDSLRDiagPixSize()));
    ui->sbScopeFL->setValue(g_AllData->getMainScopeFocalLength());
    ui->sbScopeFL->setValue(g_AllData->getMainScopeFocalLength());
    ui->sbDitherMax->setValue(g_AllData->getDitherRange(false));
    ui->sbDitherMin->setValue(g_AllData->getDitherRange(true));

    // force a few line edit fields to accept doubles or ints only ...
    ui->leLat->setValidator(new QDoubleValidator(-90, 90, 10, this));
    ui->leLong->setValidator(new QDoubleValidator(-180, 180, 10, this));
    ui->leFrameSizeX->setValidator(new QIntValidator(0,30000,this));
    ui->leFrameSizeY->setValidator(new QIntValidator(0,30000,this));
    ui->lePixelSizeX->setValidator(new QDoubleValidator(0,500,3,this));
    ui->lePixelSizeY->setValidator(new QDoubleValidator(0,500,3,this));
    ui->leRAGear->setValidator(new QDoubleValidator(-10000,10000,10,this));
    ui->leRAPlanetary->setValidator(new QDoubleValidator(-10000,10000,10,this));
    ui->leRAStepsize->setValidator(new QDoubleValidator(0,100,10,this));
    ui->leRAWorm->setValidator(new QIntValidator(0,10000,this));
    ui->leDeclGear->setValidator(new QDoubleValidator(-10000,10000,10,this));
    ui->leDeclPlanetary->setValidator(new QDoubleValidator(-10000,10000,10,this));
    ui->leDeclStepSize->setValidator(new QDoubleValidator(0,100,10,this));
    ui->leDeclWorm->setValidator(new QIntValidator(0,10000,this));

    // connecting signals and slots
    connect(this->timer, SIGNAL(timeout()), this, SLOT(updateReadings())); // this is the event queue
    connect(this->LX200Timer, SIGNAL(timeout()), this, SLOT(readLX200Port())); // this is the event for reading LX200
    connect(this->st4Timer, SIGNAL(timeout()), this, SLOT(readST4Port())); // this is the event for reading LX200
    connect(this->tempUpdateTimer, SIGNAL(timeout()), this, SLOT(getTemperature())); // this one polls temperature data from the HAT arduino
    connect(this->tcpHandBoxSendTimer, SIGNAL(timeout()), this, SLOT(sendDataToTCPHandboxSlot())); // send status of TSC to the TCP-IP handbox if connected
    connect(this->auxDriveUpdateTimer, SIGNAL(timeout()),this, SLOT(updateAuxDriveStatus())); // event for checking focusmotors and updating the GUI information
    connect(this->checkDriveTimer, SIGNAL(timeout()), this, SLOT(getDriveError())); // check the AMIS boards for internalk errors
    connect(ui->listWidgetCatalog,SIGNAL(itemClicked(QListWidgetItem*)),this,SLOT(catalogChosen(QListWidgetItem*))); // choose an available .tsc catalog
    connect(ui->listWidgetObject,SIGNAL(itemClicked(QListWidgetItem*)),this,SLOT(catalogObjectChosen())); // catalog selection
    connect(ui->listWidgetIPAddresses,SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(IPaddressChosen())); // selection of IP address for LX 200
    connect(ui->listWidgetIPAddresses_2,SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(IPaddressForHandboxChosen())); // selection of IP address for the handbox
    connect(ui->listMainCamProperties, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(mainCamPropertySelected()));
    connect(ui->listGuideCamProperties, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(guideCamPropertySelected())); // selection of INDI-Camera properties
    connect(ui->lwMainCCDTextNames, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(mainCamSetINDITextProperties())); // select a text value for display and change in INDI parameters
    connect(ui->lwGuideCCDTextNames, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(guideCamSetINDITextProperties()));
    connect(ui->lwMainCCDNumberNames, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(mainCamSetINDINumberProperties())); // select a number value for display and change in INDI parameters
    connect(ui->lwGuideCCDNumberNames, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(guideCamSetINDINumberProperties()));
    connect(ui->lwMainCCDSwitchNames, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(mainCamSetINDISwitchProperties())); // select a switch for display an change in INDI parameters
    connect(ui->lwGuideCCDSwitchNames, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(guideCamSetINDISwitchProperties()));
    connect(ui->cbLXSimpleNumbers, SIGNAL(released()),this, SLOT(LXSetNumberFormatToSimple())); // switch between simple and complex LX 200 format
    connect(ui->cbIsOnNorthernHemisphere, SIGNAL(stateChanged(int)), this, SLOT(invertRADirection())); // switch direction of RA motion for the southern hemisphere
    connect(ui->cbStoreGuideCamImgs, SIGNAL(stateChanged(int)), this, SLOT(enableCamImageStorage())); // a checkbox that starts saving all camera images in the camera-class
    connect(ui->cbMedianFilter, SIGNAL(stateChanged(int)), this, SLOT(changePrevImgProc())); // apply a 3x3 median filter to the guidestar - image
    connect(ui->cbLowPass, SIGNAL(stateChanged(int)), this, SLOT(changePrevImgProc())); // apply a 3x3 median filter to the guidestar - image
    connect(ui->cbSerialLX200Default, SIGNAL(stateChanged(int)), this, SLOT(handleSerialLXCB())); // store the checkbox state for serial LX200
    connect(ui->cbIsGEM, SIGNAL(stateChanged(int)), this, SLOT(mountIsGerman())); // toggle whether mount is a GEM or not
    connect(ui->cbMountIsEast, SIGNAL(stateChanged(int)), this, SLOT(mountIsEast())); // act whether the mount is set to east-west
    connect(ui->cbTimeFromLX200, SIGNAL(stateChanged(int)), this, SLOT(setTimeFromLX200Flag())); // check whether time from LX200 is accepted or not
    connect(ui->sbMoveSpeed, SIGNAL(valueChanged(int)),this,SLOT(changeMoveSpeed())); // set factor for faster manual motion
    connect(ui->sbFLGuideScope, SIGNAL(valueChanged(int)), this, SLOT(changeGuideScopeFL())); // spinbox for guidescope - focal length
    connect(ui->sbAMaxRA_AMIS, SIGNAL(valueChanged(int)), this, SLOT(setMaxStepperAccRA())); // process input on stepper parameters in gear-tab
    connect(ui->sbCurrMaxRA_AMIS, SIGNAL(valueChanged(double)), this, SLOT(setMaxStepperCurrentRA())); // process input on stepper parameters in gear-tab
    connect(ui->sbAMaxDecl_AMIS, SIGNAL(valueChanged(int)), this, SLOT(setMaxStepperAccDecl())); // process input on stepper parameters in gear-tab
    connect(ui->sbCurrMaxDecl_AMIS, SIGNAL(valueChanged(double)), this, SLOT(setMaxStepperCurrentDecl())); // process input on stepper parameters in gear-tab
    connect(ui->sbNoFlip, SIGNAL(valueChanged(int)), this, SLOT(setDecForNoFlip())); // store the maximum declination for not doing a meridian flip
    connect(ui->sbPSSearchRad, SIGNAL(valueChanged(double)), this, SLOT(psSetSearchRadiusForPS())); // store the search radius for plate solving
    connect(ui->rbCorrSpeed,SIGNAL(released()), this, SLOT(setCorrectionSpeed())); // set speed for slow manual motion
    connect(ui->rbMoveSpeed,SIGNAL(released()), this, SLOT(setMoveSpeed())); // set speed for faster manual motion
    connect(ui->rbFOVStd, SIGNAL(released()), this, SLOT(setRegularFOV())); // guidestar window set to 180x180 pixels
    connect(ui->rbFOVHalf, SIGNAL(released()), this, SLOT(setHalfFOV())); // guidestar window set to 90x90 pixels
    connect(ui->rbFOVDbl, SIGNAL(released()), this, SLOT(setDoubleFOV())); // guidestar window set to 360x360 pixels
    connect(ui->rbSiderealSpeed, SIGNAL(released()), this, SLOT(setTrackingRate())); // set sidereal tracking rate
    connect(ui->rbLunarSpeed, SIGNAL(released()), this, SLOT(setTrackingRate())); // set lunar tracking rate
    connect(ui->rbSolarSpeed, SIGNAL(released()),this, SLOT(setTrackingRate())); // set solar tracking rate
    connect(ui->rbMainCCD, SIGNAL(released()),this, SLOT(selectCameraTypes()));
    connect(ui->rbGuiderCCD, SIGNAL(released()), this, SLOT(selectCameraTypes())); // select if the first camera connected is guiderr or main ccd
    connect(ui->pbParkToSouthHorizon, SIGNAL(clicked()), this, SLOT(presetParkingPositionSouthH())); // set a parking position
    connect(ui->pbParkPolaris, SIGNAL(clicked()), this, SLOT(presetParkingPositionPolaris())); // set a parking position
    connect(ui->pbChoosePathToFITS, SIGNAL(clicked()), this, SLOT(psChooseFITSDirectory())); // choose a directory for storing FITS images for plate solving
    connect(ui->rbNoFinderFocuser, SIGNAL(released()),this, SLOT(storeAuxBoardParams()));
    connect(ui->rbNo1FinderFocuser , SIGNAL(released()),this, SLOT(storeAuxBoardParams()));
    connect(ui->rbNo2FinderFocuser, SIGNAL(released()),this, SLOT(storeAuxBoardParams())); // store the auxiliary drive which is used for the guider when the drive no. is changed
    connect(ui->hsThreshold,SIGNAL(valueChanged(int)), this, SLOT(changePrevImgProc())); // change threshold for selecting a guidestar
    connect(ui->hsIContrast ,SIGNAL(valueChanged(int)), this, SLOT(changePrevImgProc())); // change contrast for selecting a guidestar
    connect(ui->hsIBrightness ,SIGNAL(valueChanged(int)), this, SLOT(changePrevImgProc())); // change brightness for selecting a guidestar
    connect(ui->pbExit,SIGNAL(clicked()), this, SLOT(shutDownProgram())); // this kills the program, including killing the drives
    connect(ui->pbConnectToServer,SIGNAL(clicked()),this, SLOT(setINDISAddrAndPort())); // connects to the INDI server at the given address ...
    connect(ui->pbSelectGuiderCCD, SIGNAL(clicked()), this, SLOT(selectGuiderCamDriverName()));
    connect(ui->pbMainCCDSetINDISwitch , SIGNAL(clicked()), this, SLOT(mainCamSendINDISwitch())); // send a value to INDI server
    connect(ui->pbGuideCCDSetINDISwitch , SIGNAL(clicked()), this, SLOT(guideCamSendINDISwitch())); // send a value to INDI server
    connect(ui->pbSelectMainCCD, SIGNAL(clicked()), this, SLOT(selectMainCamDriverName()));
    connect(ui->pbKillINDIServer, SIGNAL(clicked()),this, SLOT(killRunningINDIServer())); // a button that kill running INDI servers ...
    connect(ui->pbDisconnectFromServer, SIGNAL(clicked()), this, SLOT(disconnectFromINDIServer())); // disconnects from INDI server
    connect(ui->pbExpose, SIGNAL(clicked()), this, SLOT(startCCDAcquisition())); // start acquiring images from the guidecam. a signal is emitted if an image arrived.
    connect(ui->pbStopExposure, SIGNAL(clicked()), this, SLOT(stopCCDAcquisition())); // just set the local flag on ccd-acquisition so that no new image is polled in "displayGuideCamImage".
    connect(ui->pbClearINDILog,SIGNAL(clicked()), this, SLOT(clearINDILog())); // clear the textbox for INDI server messages
    connect(ui->pbSync, SIGNAL(clicked()), this, SLOT(syncMount())); // reset the current position and global timer, and set the global mount position to the actual coordinates
    connect(ui->pbStoreGears, SIGNAL(clicked()), this, SLOT(storeGearData())); // well - take the data from the dialog and store them in the .tsp file and in g_AllData
    connect(ui->pbStoreSiteData, SIGNAL(clicked()), this, SLOT(storeSiteData())); // store information the observatory position and so on
    connect(ui->pbStartTracking, SIGNAL(clicked()),this,SLOT(startRATracking())); // start earth motion compensation in RA
    connect(ui->pbStopTracking, SIGNAL(clicked()),this,SLOT(stopRATracking())); // stop earth motion compensation in RA
    connect(ui->pbDeclUp, SIGNAL(clicked()),this,SLOT(declinationMoveHandboxUp())); // manual motion of the handbox - decl up
    connect(ui->pbDeclDown, SIGNAL(clicked()),this,SLOT(declinationMoveHandboxDown())); // manual motion of the handbox - decl down
    connect(ui->pbRAPlus, SIGNAL(clicked()),this,SLOT(RAMoveHandboxFwd())); // manual motion of the handbox - ra towards sunset
    connect(ui->pbRAMinus, SIGNAL(clicked()),this,SLOT(RAMoveHandboxBwd())); // manual motion of the handbox - ra towards dawn
    connect(ui->pbStoreDrive, SIGNAL(clicked()), this, SLOT(storeDriveData())); // store data to preferences
    connect(ui->pbGoTo, SIGNAL(clicked()),this, SLOT(startGoToObject())); // start the slew routine
    connect(ui->pbLX200Active, SIGNAL(clicked()), this, SLOT(switchToLX200())); // open the serial port for LX 200
    connect(ui->pbStartINDIServer, SIGNAL(clicked()), this, SLOT(deployINDICommand())); // call a system command to start an INDI server with given driver parameters
    connect(ui->pbStop1, SIGNAL(clicked()), this, SLOT(emergencyStop())); // kill all motion immediately
    connect(ui->pbStop2, SIGNAL(clicked()), this, SLOT(emergencyStop())); // kill all motion immediately
    connect(ui->pbTerminateCal, SIGNAL(clicked()), this, SLOT(terminateGuiderCalibration())); // kill all motion immediately
    connect(ui->pbEnableTCP, SIGNAL(clicked()), this, SLOT(connectToIPSocket())); // connect to a LX 200 socket
    connect(ui->pbDisableTCP, SIGNAL(clicked()), this, SLOT(disconnectFromIPSocket())); // disconnect from LX 200 socket
    connect(ui->pbTCPHBEnable, SIGNAL(clicked()), this, SLOT(connectHandboxToIPSocket())); // connect to a IP socket for the handbox
    connect(ui->pbTCPHBDisable, SIGNAL(clicked()), this, SLOT(disconnectHandboxFromIPSocket())); // disconnect the TCP/IP handbox und shut down server
    connect(ui->pbClearLXLog, SIGNAL(clicked()), this, SLOT(clearLXLog())); // delete the log of LX200 commands
    connect(ui->pbSelectGuideStar, SIGNAL(clicked()), this, SLOT(selectGuideStar())); // select a guide star defined by crosshair in the QDisplay - widget
    connect(ui->pbConfirmGuideStar, SIGNAL(clicked()), this, SLOT(confirmGuideStar())); // just disables the follwing GUI elements in the autoguiding process
    connect(ui->pbGuiding,SIGNAL(clicked()), this, SLOT(doAutoGuiding())); // instantiate all variables for autoguiding and set a flag that takes care of correction in "displayGuideCamImage" and "correctGuideStarPosition"
    connect(ui->pbStoreFL, SIGNAL(clicked()), this, SLOT(storeGuideScopeFL())); // store focal length of guidescope to preferences
    connect(ui->pbTCPHBKillMotion, SIGNAL(clicked()), this, SLOT(killHandBoxMotion())); // terminates handbox motion if handbox TCP-connection is lost
    connect(ui->pbTrainAxes, SIGNAL(clicked()),this, SLOT(calibrateAutoGuider())); // find rotation and stepwidth for autoguiding
    connect(ui->pbSkipCalibration, SIGNAL(clicked()), this, SLOT(skipCalibration(void))); // does a dummy calibration for autoguiding - for testing purposes
    connect(ui->pbResetGuiding, SIGNAL(clicked()), this, SLOT(resetGuidingCalibration())); // reset autoguider calibration
    connect(ui->pbResetGdErr, SIGNAL(clicked()), this, SLOT(resetGuidingError())); // reset autoguider guiding error
    connect(ui->pbStartST4, SIGNAL(clicked()),this, SLOT(startST4Guiding())); // start ST4 pulse guiding
    connect(ui->pbStopST4, SIGNAL(clicked()),this, SLOT(stopST4Guiding())); // stop ST4 pulse guiding
    connect(ui->pbDSLRSingleShot, SIGNAL(clicked()), this, SLOT(handleDSLRSingleExposure())); // start a dslr exposure
    connect(ui->pbDSLRStartSeries, SIGNAL(clicked()), this, SLOT(startDSLRSeries())); // start a series of DSLR exposures
    connect(ui->pbDSLRStopSeries, SIGNAL(clicked()), this, SLOT(terminateDSLRSeries())); // terminate a dslr series exposure early
    connect(ui->pbConveyCoordinates, SIGNAL(clicked()), this, SLOT(transferCoordinates())); // slot that transfers coordinates to the controller
    connect(ui->pbDSLRTerminateExposure, SIGNAL(clicked()), this, SLOT(terminateDSLRSingleShot())); // stop a single DSLR exposure
    connect(ui->pbStoreAuxData, SIGNAL(clicked()), this, SLOT(storeAuxBoardParams())); // stores parameters for the auxiliary drive
    connect(ui->pbStopAuxDrives, SIGNAL(clicked()), this, SLOT(emergencyStopAuxDrives())); // stop all auxiliary drives
    connect(ui->pbFwdMovAux1, SIGNAL(clicked()), this, SLOT(mvAux1FwdFull())); // move aux drive 1 forward
    connect(ui->pbBwdMovAux1, SIGNAL(clicked()), this, SLOT(mvAux1BwdFull())); // move aux drive 1 bward
    connect(ui->pbFwdMovAux2, SIGNAL(clicked()), this, SLOT(mvAux2FwdFull())); // move aux drive 2 forward
    connect(ui->pbBwdMovAux2, SIGNAL(clicked()), this, SLOT(mvAux2BwdFull())); // move aux drive 2 bward
    connect(ui->pbFwdMovAux1_Tenth, SIGNAL(clicked()), this, SLOT(mvAux1FwdSmall())); // move aux drive 1 forward
    connect(ui->pbBwdMovAux1_Tenth, SIGNAL(clicked()), this, SLOT(mvAux1BwdSmall())); // move aux drive 1 bward
    connect(ui->pbFwdMovAux2_Tenth, SIGNAL(clicked()), this, SLOT(mvAux2FwdSmall())); // move aux drive 2 forward
    connect(ui->pbBwdMovAux2_Tenth, SIGNAL(clicked()), this, SLOT(mvAux2BwdSmall())); // move aux drive 2 bward
    connect(ui->pbFwdMovAux1_Thirth, SIGNAL(clicked()), this, SLOT(mvAux1FwdTiny())); // move aux drive 1 forward
    connect(ui->pbBwdMovAux1_Thirth, SIGNAL(clicked()), this, SLOT(mvAux1BwdTiny())); // move aux drive 1 bward
    connect(ui->pbFwdMovAux2_Thirth, SIGNAL(clicked()), this, SLOT(mvAux2FwdTiny())); // move aux drive 2 forward
    connect(ui->pbBwdMovAux2_Thirth, SIGNAL(clicked()), this, SLOT(mvAux2BwdTiny())); // move aux drive 2 bward 
    connect(ui->pbGdFocBigFwd, SIGNAL(clicked()), this, SLOT(mvGuideAuxFwdFull())); // move guide focuser drive 1 forward
    connect(ui->pbGdFocBigBwd, SIGNAL(clicked()), this, SLOT(mvGuideAuxBwdFull())); // move guide focuser drive 1 bward
    connect(ui->pbGdFocMedFwd, SIGNAL(clicked()), this, SLOT(mvGuideAuxFwdSmall())); // move guide focuser drive 1 forward
    connect(ui->pbGdFocMedBwd, SIGNAL(clicked()), this, SLOT(mvGuideAuxBwdSmall())); // move guide focuser drive 1 bward
    connect(ui->pbGdFocSmallFwd, SIGNAL(clicked()), this, SLOT(mvGuideAuxFwdTiny())); // move guide focuser drive 1 forward
    connect(ui->pbGdFocSmallBwd, SIGNAL(clicked()), this, SLOT(mvGuideAuxBwdTiny())); // move guide focuser drive 1 bward
    connect(ui->pbStoreDSLRSettings, SIGNAL(clicked()), this, SLOT(storeDSLRSettingsForDithering())); // store DSLR pixel size and main scope focal length for dithering
    connect(ui->pbStoreSettingsPS, SIGNAL(clicked()), this, SLOT(storeSettingsForPlateSolving())); // slot for stroing the settings in platesolving
    connect(ui->pbStoreSpeeds, SIGNAL(clicked()), this, SLOT(storeHandBoxSpeeds())); // store the goto and motion speeds for the handbox
    connect(ui->pbStorePark, SIGNAL(clicked()), this, SLOT(determineParkingPosition())); // store the actual position as parking position
    connect(ui->pbGoToPark, SIGNAL(clicked()), this, SLOT(gotoParkPosition())); // move the telescope to the parking position and stop motion
    connect(ui->pbSyncToPark, SIGNAL(clicked()), this, SLOT(syncParkPosition())); // if the scope is parked, sync it to the known parking position
    connect(ui->pbTakeImagePS, SIGNAL(clicked()), this, SLOT(psTakeImage())); // take an image for platesolving
    connect(ui->pbSolveFieldPS, SIGNAL(clicked()), this, SLOT(psStartSolving())); // issue a command to astrometry.net
    connect(ui->pbKillAMetry, SIGNAL(clicked()), this, SLOT(psKillAstrometryNet())); // stop astrometry.net
    connect(ui->pbSyncPS, SIGNAL(clicked()), this, SLOT(syncPSCoordinates())); // sync to coordinates from platesolving
    connect(ui->pbMainCCDNumberValueSet, SIGNAL(clicked()), this, SLOT(mainCamSendINDINumber())); // sends a new number value to INDI server
    connect(ui->pbMainCCDNumberValueSend, SIGNAL(clicked()), this, SLOT(mainCamSetINDINumberOnServer())); // sets the number value on the INDI server
    connect(ui->pbMainCCDTextValueSet, SIGNAL(clicked()), this, SLOT(mainCamSendINDIText())); // sends text to INDI
    connect(ui->pbGuideCCDNumberValueSet, SIGNAL(clicked()), this, SLOT(guideCamSendINDINumber())); // sends a new number value to INDI server
    connect(ui->pbGuideCCDNumberValueSend, SIGNAL(clicked()), this, SLOT(guideCamSetINDINumberOnServer()));
    connect(ui->pbGuideCCDTextValueSet, SIGNAL(clicked()), this, SLOT(guideCamSendINDIText())); // sends text to INDI
    connect(ui->pbStoreMainCCDProps, SIGNAL(clicked()), this, SLOT(saveMainCCDConfig())); // issues a CONFIG_SAVE switch to INDIserver
    connect(ui->pbStoreGuiderCCDProps,SIGNAL(clicked()), this, SLOT(saveGuideCCDConfig())); // issues a CONFIG_SAVE switch to INDIserver
    connect(this, SIGNAL(dslrExposureDone()), this, SLOT(takeNextExposureInSeries()),Qt::QueuedConnection); // this is called when an exposure is done; if a series is taken, the next exposure is triggered ...
    connect(this->camView,SIGNAL(currentViewStatusSignal(QPointF)),this->camView,SLOT(currentViewStatusSlot(QPointF)),Qt::QueuedConnection); // position the crosshair in the camera view by mouse...
    connect(this->guiding,SIGNAL(determinedGuideStarCentroid()), this->camView,SLOT(currentViewStatusSlot()),Qt::QueuedConnection); // an overload of the precious slot that allows for positioning the crosshair after a centroid was computed during guiding...
    connect(this->camera_client,SIGNAL(imageAvailable(QPixmap*)),this,SLOT(displayGuideCamImage(QPixmap*)),Qt::QueuedConnection); // display image from ccd if one was received from INDI; also takes care of autoguiding. triggered by signal
    connect(this->camera_client, SIGNAL(mainCCDImageAvailable(QPixmap*)), this, SLOT(displayMainCamImage(QPixmap*)), Qt::QueuedConnection); // display an image of the main camera for platesolving
    connect(this->camera_client,SIGNAL(messageFromINDIAvailable()),this,SLOT(handleServerMessage()),Qt::QueuedConnection); // display messages from INDI if signal was received
    connect(this->camera_client,SIGNAL(newPropertyListArrived()), this, SLOT(loadPropertyList()),Qt::QueuedConnection); // called when a new property comes in
    connect(this->camera_client,SIGNAL(numberSetOnServerMainCCD()), this, SLOT(indicateNumberOnINDIServerMainCCD()),Qt::QueuedConnection);
    connect(this->camera_client,SIGNAL(textSetOnServerMainCCD()), this, SLOT(indicateTextOnINDIServerMainCCD()),Qt::QueuedConnection);
    connect(this->camera_client,SIGNAL(switchSetOnServerMainCCD()), this, SLOT(indicateSwitchOnINDIServerMainCCD()),Qt::QueuedConnection); // setting checkboxes when new values are stored in the INDI server
    connect(this->camera_client,SIGNAL(numberSetOnServerGuiderCCD()), this, SLOT(indicateNumberOnINDIServerGuiderCCD()),Qt::QueuedConnection);
    connect(this->camera_client,SIGNAL(textSetOnServerGuiderCCD()), this, SLOT(indicateTextOnINDIServerGuiderCCD()),Qt::QueuedConnection);
    connect(this->camera_client,SIGNAL(switchSetOnServerGuiderCCD()), this, SLOT(indicateSwitchOnINDIServerGuiderCCD()),Qt::QueuedConnection); // setting checkboxes when new values are stored in the INDI server
    connect(this->camera_client,SIGNAL(saveFunctionNotAvailable()), this, SLOT(deployINDIMsgDlg()),Qt::QueuedConnection); // open a dialog box if INDI server cannot save configurations
    connect(this->guiding,SIGNAL(guideImagePreviewAvailable()),this,SLOT(displayGuideStarPreview()),Qt::QueuedConnection); // handle preview of the processed guidestar image
    connect(this, SIGNAL(tcpHandboxDataReceived()), this, SLOT(handleHandbox()),Qt::QueuedConnection); // handle data comming from the TCP/IP handbox
    connect(this->LXServer,SIGNAL(newConnection()),this,SLOT(establishLX200IPLink()),Qt::QueuedConnection); // establish a link vian LAN/WLAN to a planetarium program via TCP/IP
    connect(this->HBServer, SIGNAL(newConnection()), this, SLOT(establishHBIPLink()),Qt::QueuedConnection); // same as abov for the TCP handbox
    connect(this->astroMetryProcess, SIGNAL(finished(int, QProcess::ExitStatus)), this, SLOT(psHandleEndOfAstronomyNetProcess(int, QProcess::ExitStatus))); // handle the end of platesolving
    connectLX200Events(true); // call all connects for LX200 functions
    this->killRunningINDIServer(); // find out about running INDI servers and kill them
    ui->lcdPulseGuideDuration->display(pulseGuideDuration);
    ui->cbLowPass->setEnabled(true); // this is probably a real bug in qtdesigner ... no way to enable that checkbox ...
    ui->fSwitchMainCCD->setEnabled(true);
    ui->fNumberMainCCD->setEnabled(true);
    ui->fTextMainCCD->setEnabled(true);
    ui->fLightMainCCD->setEnabled(false);  // same as above
    ui->pbSyncToPark->setEnabled(true); // same as above
    this->StepperDriveRA->stopDrive();
    this->StepperDriveDecl->stopDrive(); // just to kill all jobs that may lurk in the muproc ...
    this->currentRAString = new QString();
    this->currentDeclString = new QString();
    this->currentHAString = new QString();
    this->coordString = new QString();
    // read the last state (east/west) of the GEM
    std::string line;   // define a line that is read until \n is encountered
    std::ifstream infile(".GEMState.tsl");  // read that special file ...
    if (infile.is_open()) {
        std::getline(infile, line);
        std::istringstream isEastFlag(line);   // convert 'line' to a stream so that the first line
        isEastFlag >> isEastFlagFromFile;
        infile.close();
        if (isEastFlagFromFile == 1) {
            ui->cbMountIsEast->setChecked(true);
            g_AllData->setMFlipParams(1,true);
        } else {
            ui->cbMountIsEast->setChecked(false);
            g_AllData->setMFlipParams(1,false);
        }
    }
}

//------------------------------------------------------------------
// initialisation of the stepper driver hardware
short MainWindow::initiateStepperDrivers(void) {
    double draccRA, draccDecl, drcurrRA, drcurrDecl; // local values on drive acceleration and so on...

    draccRA = g_AllData->getDriveParams(0,1);
    draccDecl = g_AllData->getDriveParams(1,1);
    drcurrRA = g_AllData->getDriveParams(0,2);
    drcurrDecl = g_AllData->getDriveParams(1,2); // retrieving acceleration and maximum current for the phidget boards
    g_AllData->setDriveData(0,0);
    g_AllData->setDriveData(1,0);
    amisInterface = new usbCommunications(0x16c0);
    StepperDriveRA = new QtContinuousStepper();
    StepperDriveRA->changeMicroSteps(g_AllData->getMicroSteppingRatio(0));
    this->StepperDriveRA->setGearRatioAndMicrosteps(g_AllData->getGearData(0 )*g_AllData->getGearData(1 )*
                                                    g_AllData->getGearData(2 )/g_AllData->getGearData(3 ),
                                                    g_AllData->getMicroSteppingRatio(0));
    this->StepperDriveRA->setInitialParamsAndComputeBaseSpeed(draccRA,drcurrRA); // setting initial parameters for the ra drive
    StepperDriveDecl = new QtKineticStepper();
    StepperDriveDecl->changeMicroSteps(g_AllData->getMicroSteppingRatio(0));
    this->StepperDriveDecl->setGearRatioAndMicrosteps(g_AllData->getGearData(4 )*g_AllData->getGearData(5 )*
                                                      g_AllData->getGearData(6 )/g_AllData->getGearData(7 ),
                                                      g_AllData->getMicroSteppingRatio(0));
    this->StepperDriveDecl->setInitialParamsAndComputeBaseSpeed(draccDecl,drcurrDecl); // setting initial parameters for the declination drive
    return 0;
}
//------------------------------------------------------------------
// a function to call all connects for the LX200 class; mainly inserted for convenience. first called in
// the constructor.
void MainWindow::connectLX200Events(bool doConnect) {
    if (doConnect == true) {
        connect(this->lx200Comm,SIGNAL(RS232moveEast()), this, SLOT(LXmoveEast()),Qt::QueuedConnection);
        connect(this->lx200Comm,SIGNAL(RS232moveWest()), this, SLOT(LXmoveWest()),Qt::QueuedConnection);
        connect(this->lx200Comm,SIGNAL(RS232moveNorth()), this, SLOT(LXmoveNorth()),Qt::QueuedConnection);
        connect(this->lx200Comm,SIGNAL(RS232moveSouth()), this, SLOT(LXmoveSouth()),Qt::QueuedConnection);
        connect(this->lx200Comm,SIGNAL(RS232stopMoveEast()), this, SLOT(LXstopMoveEast()),Qt::QueuedConnection);
        connect(this->lx200Comm,SIGNAL(RS232stopMoveWest()), this, SLOT(LXstopMoveWest()),Qt::QueuedConnection);
        connect(this->lx200Comm,SIGNAL(RS232stopMoveNorth()), this, SLOT(LXstopMoveNorth()),Qt::QueuedConnection);
        connect(this->lx200Comm,SIGNAL(RS232stopMoveSouth()), this, SLOT(LXstopMoveSouth()),Qt::QueuedConnection); // LX 200 handbox commands
        connect(this->lx200Comm,SIGNAL(RS232stopMotion()), this, SLOT(LXstopMotion()),Qt::QueuedConnection); // total stop of all motion by LX 200
        connect(this->lx200Comm,SIGNAL(RS232guideSpeed()), this, SLOT(LXslowSpeed()),Qt::QueuedConnection);
        connect(this->lx200Comm,SIGNAL(RS232centerSpeed()), this, SLOT(LXslowSpeed()),Qt::QueuedConnection);
        connect(this->lx200Comm,SIGNAL(RS232findSpeed()), this, SLOT(LXhiSpeed()),Qt::QueuedConnection);
        connect(this->lx200Comm,SIGNAL(RS232gotoSpeed()), this, SLOT(LXhiSpeed()),Qt::QueuedConnection); // LX 200 knows four speeds, we only know 2 - sidereal correction and fast motion
        connect(this->lx200Comm,SIGNAL(RS232sync()),this,SLOT(LXsyncMount()),Qt::QueuedConnection); // LX 200 sync
        connect(this->lx200Comm,SIGNAL(RS232slew()),this,SLOT(LXslewMount()),Qt::QueuedConnection); // LX 200 slew
        connect(this->lx200Comm,SIGNAL(RS232CommandReceived()),this, SLOT(logLX200IncomingCmds()),Qt::QueuedConnection); // write incoming command from LX 200 to log
        connect(this->lx200Comm,SIGNAL(logRASent()),this, SLOT(logLX200OutgoingCmdsRA()),Qt::QueuedConnection); // receive RA from LX 200 and log it
        connect(this->lx200Comm,SIGNAL(logDeclSent()),this, SLOT(logLX200OutgoingCmdsDecl()),Qt::QueuedConnection); // receive decl from LX 200 and log it
        connect(this->lx200Comm,SIGNAL(logCommandSent()),this, SLOT(logLX200OutgoingCmds()),Qt::QueuedConnection); // write outgoing command from LX 200 to log
        connect(this->lx200Comm,SIGNAL(polarAlignmentSignal()), this, SLOT(sendPolarAlignmentCommand()),Qt::QueuedConnection); // send a "P#" upon establishing conntact via classic LX200 over the TCP/IP socket ...
        connect(this->lx200Comm,SIGNAL(clientRASent(QString*)), this, SLOT(handleRAviaTCP(QString*)),Qt::QueuedConnection);
        connect(this->lx200Comm,SIGNAL(clientDeclSent(QString*)), this, SLOT(handleDeclviaTCP(QString*)),Qt::QueuedConnection);
        connect(this->lx200Comm,SIGNAL(clientCommandSent(QString*)), this, SLOT(handleCommandviaTCP(QString*)),Qt::QueuedConnection);
        connect(this->lx200Comm,SIGNAL(localizationSet()), this, SLOT(updateLocalization()), Qt::QueuedConnection);
    }
}

//------------------------------------------------------------------
// destructor - hopefully kills all local and global instances
MainWindow::~MainWindow() {
    this->shutDownProgram();
    delete ui;
    exit(0);
}

//------------------------------------------------------------------
// during each run of the event queue the drive state needs to be checked

bool MainWindow::isDriveActive(bool isRA) {
    bool state=false;

    amisInterface->sendCommand("f0", isRA);
    if (amisInterface->getReply(isRA).toLong() == 0) { // checking whether the drives are in motion
        state = false;
    } else {
        state = true;
    }
    return state;
}

//------------------------------------------------------------------
// the main event queue, triggered by this->timer
void MainWindow::updateReadings() {
    qint64 topicalTime; // g_AllData contains an monotonic global timer that is reset if a sync occcurs
    double relativeTravelRA, relativeTravelDecl,totalGearRatio, hourAngleForDisplay; // a few helpers
    bool wasInGoTo = false, isInGoTo = false, isEast;

    isEast = g_AllData->getMFlipParams(1); // store east/west flag for GEMs in case a meridian flip occurs. after a flip,
    // the "isEast" checkbox is deactivated.

    if (this->guidingState.systemIsCalibrated == true) {
        ui->cbAutoguiderIsCalibrated->setChecked(true);
    } else {
        ui->cbAutoguiderIsCalibrated->setChecked(false);
    }
    if (this->guidingState.guidingIsOn == true) {
        ui->cbDither->setEnabled(true);
    } else {
        ui->cbDither->setEnabled(false);
    }
    this->updateTimeAndDate();

    if (this->tcpHandboxIsConnected == true) {
        if (this->HBSocket->bytesAvailable()) {
            this->readTCPHandboxData();
        }
    }

    if (this->ccdGuiderCameraIsAcquiring == true) { // the to be analysed by the opencv in guiding is checked for saturated pixels
        ui->cbSaturation->setChecked(this->guiding->isPixelAtSaturation());
    }

    if (this->dslrStates.dslrExposureIsRunning == true) { // check a timer and update display of the remaining time ...
        this->updateDSLRGUIAndCountdown();
    }
    if (this->mountMotion.RATrackingIsOn == true) { // standard mode - mount compensates for earth motion
        topicalTime = g_AllData->getTimeSinceLastSync() - this->mountMotion.RAtrackingElapsedTimeInMS; // check the monotonic timer
        this->mountMotion.RAtrackingElapsedTimeInMS+=topicalTime; // total time elapsed in tracking mode
        totalGearRatio = g_AllData->getGearData(0 )*g_AllData->getGearData(1 )*g_AllData->getGearData(2 ); // gear ratio in RA
        relativeTravelRA= this->StepperDriveRA->getKineticsFromController(3)*topicalTime*g_AllData->getGearData(3 )/
                (1000.0*g_AllData->getMicroSteppingRatio((short)this->raState)*totalGearRatio); // compute travel in decimal degrees
        g_AllData->incrementActualScopePosition(relativeTravelRA, 0.0); // update position in global struct on RA
    }
    if (this->StepperDriveRA->getStopped() == true) { // handle updates if the ra-stepper is not moving at all
        topicalTime = g_AllData->getTimeSinceLastSync() - this->mountMotion.RAtrackingElapsedTimeInMS; // check the monotonic timer
        this->mountMotion.RAtrackingElapsedTimeInMS+=topicalTime; // total time elapsed in tracking mode
        relativeTravelRA = 0;
        g_AllData->incrementActualScopePosition(relativeTravelRA, 0.0); // update position in global struct on RA
    }

    if (this->mountMotion.RADriveIsMoving == true) { // mount moves at non-sidereal rate - but not in GOTO
        topicalTime = g_AllData->getTimeSinceLastSync() - this->mountMotion.RAMoveElapsedTimeInMS;
        this->mountMotion.RAMoveElapsedTimeInMS+=topicalTime;
        totalGearRatio = g_AllData->getGearData(0)*g_AllData->getGearData(1)*g_AllData->getGearData(2);
        relativeTravelRA=this->mountMotion.RADriveDirection*
                this->StepperDriveRA->getKineticsFromController(3)*topicalTime*g_AllData->getGearData(3 )/
                (1000.0*g_AllData->getMicroSteppingRatio((short)(this->raState))*totalGearRatio);
        g_AllData->incrementActualScopePosition(relativeTravelRA, 0.0);  // same as above - compute travel in decimal degrees and update it
        if (this->StepperDriveRA->hasHBoxSlewEnded() == true) {
            this->mountMotion.RADriveIsMoving = false;
            if (this->mountMotion.RATrackingIsOn == false) {
                this->setControlsForRATravel(true);
            }
            this->startRATracking();
            ui->pbRAMinus->setEnabled(1);
            ui->pbRAPlus->setEnabled(1);
            ui->rbCorrSpeed->setEnabled(true);
            ui->rbMoveSpeed->setEnabled(true);
            if (ui->rbMoveSpeed->isChecked()==false) {
                ui->sbMoveSpeed->setEnabled(true);
            }
        } // a lot of code for an unlikely situation. after 180 degrees, the mount simply resumes tracking
    }

    if (this->mountMotion.DeclDriveIsMoving == true) { // now, the declination drive is also active; it does not track, therefore we have a copy of the above section, more or less
        topicalTime = g_AllData->getTimeSinceLastSync() - this->mountMotion.DeclMoveElapsedTimeInMS;
        this->mountMotion.DeclMoveElapsedTimeInMS+=topicalTime;
        totalGearRatio = g_AllData->getGearData(4 )*g_AllData->getGearData(5 )*g_AllData->getGearData(6 );
        relativeTravelDecl= this->mountMotion.DeclDriveDirection*g_AllData->getMFlipDecSign()*
                this->StepperDriveDecl->getKineticsFromController(3)*topicalTime*g_AllData->getGearData(7 )/
                (1000.0*g_AllData->getMicroSteppingRatio((short)this->deState)*totalGearRatio);
        if (g_AllData->incrementActualScopePosition(0.0, relativeTravelDecl) == true) { // update the declination position and check whether a meridian flip took place
            if (g_AllData->getMFlipParams(0) == true) { // ... if the mount is GEM and the MF is on
                if (g_AllData->getMFlipParams(1) == true) { // ... and if mount is east
                    ui->cbMountIsEast->setChecked(false); // invert east/west state
                } else {
                    ui->cbMountIsEast->setChecked(true);
                }
            }
        }
        if (this->StepperDriveDecl->hasHBoxSlewEnded() == true) {
            // same as above; end of handbox slew of 180 degrees has to be handled like pressing a stop button
            this->mountMotion.DeclDriveIsMoving = false;
            ui->pbDeclUp->setEnabled(true);
            ui->pbDeclDown->setEnabled(true);
            this->setControlsForDeclTravel(true);
            if (ui->rbMoveSpeed->isChecked()==false) {
                ui->sbMoveSpeed->setEnabled(true);
            } else {
                ui->sbMoveSpeed->setEnabled(false);
            }
        } // after 180 degrees, the declination travel simply stops
    }

    if ((this->mountMotion.GoToIsActiveInRA == true) || (this->mountMotion.GoToIsActiveInDecl == true)) {
        wasInGoTo = true;
    } // checks whether GoTo is active

    if ((this->mountMotion.GoToIsActiveInRA==true) || (this->mountMotion.GoToIsActiveInDecl==true)) { // the mount is slewing. slew is not complete as either RA or decl are in slew mode - or both

        if ((this->mountMotion.RATrackingIsOn == false) && (this->mountMotion.GoToIsActiveInRA == false) && (this->isInParking == false)) {
            this->startRATracking(); // start tracking if RA slew ended
        }
        if (this->isDriveActive(true) == false) {
            this->mountMotion.GoToIsActiveInRA = false;
        }
        if (this->isDriveActive(false) == false) {
            this->mountMotion.GoToIsActiveInDecl = false;
        }
        ui->lcdGotoTime->display(round((this->gotoETA-this->elapsedGoToTime->elapsed())*0.001));
        if (this->mountMotion.GoToIsActiveInRA==true) {
            topicalTime = g_AllData->getTimeSinceLastSync() - this->mountMotion.RAGoToElapsedTimeInMS;
            this->mountMotion.RAGoToElapsedTimeInMS+=topicalTime;
            totalGearRatio = g_AllData->getGearData(0 )*g_AllData->getGearData(1 )*g_AllData->getGearData(2 );
            relativeTravelRA=this->mountMotion.RADriveDirection*
                    this->approximateGOTOSpeedRA*topicalTime*g_AllData->getGearData(3 )/
                    (1000.0*g_AllData->getMicroSteppingRatio((short)this->raState)*totalGearRatio);
            g_AllData->incrementActualScopePosition(relativeTravelRA, 0.0);
        }
        if (this->mountMotion.GoToIsActiveInDecl==true) {
            topicalTime = g_AllData->getTimeSinceLastSync() - this->mountMotion.DeclGoToElapsedTimeInMS;
            this->mountMotion.DeclGoToElapsedTimeInMS+=topicalTime;
            totalGearRatio = g_AllData->getGearData(4)*g_AllData->getGearData(5)*g_AllData->getGearData(6);
            relativeTravelDecl= this->mountMotion.DeclDriveDirection*g_AllData->getMFlipDecSign()*
                    this->approximateGOTOSpeedDecl*topicalTime*g_AllData->getGearData(7)/
                    (1000.0*g_AllData->getMicroSteppingRatio((short)this->deState)*totalGearRatio);
            if (g_AllData->incrementActualScopePosition(0.0, relativeTravelDecl) == true) { // update the declination position and check whether a meridian flip took place
                if (g_AllData->getMFlipParams(0) == true) { // ... if the mount is GEM and the MF is on
                    if (g_AllData->getMFlipParams(1) == true) { // ... and if mount is east
                        ui->cbMountIsEast->setChecked(false); // invert east/west state
                    } else {
                        ui->cbMountIsEast->setChecked(true);
                    }
                }
            }
        }
    }

    this->currentRAString->clear(); // compose the right asccension as string - similar to the routine in the LX200 class
    this->currentRAString->append(this->generateCoordinateString(g_AllData->getActualScopePosition(2),true));
    this->currentDeclString->clear();
    this->currentDeclString->append(this->generateCoordinateString(g_AllData->getActualScopePosition(1), false));
    ui->leRightAscension->setText(*currentRAString);
    ui->lePSRA->setText(*currentRAString);
    ui->leDecl->setText(*currentDeclString);
    ui->lePSDecl->setText(*currentDeclString);
    hourAngleForDisplay=(g_AllData->getLocalSTime()*15 - g_AllData->getActualScopePosition(2));
    while (hourAngleForDisplay < 0) {
        hourAngleForDisplay += 360;
    }
    while (hourAngleForDisplay > 360) {
        hourAngleForDisplay -= 360;
    }
    this->currentHAString->clear();
    this->currentHAString->append(this->generateCoordinateString(hourAngleForDisplay, true)); // hour angle is converted like RA
    ui->leHourAngle->setText(*currentHAString);
    // finally, the actual scope position is updated in the GUI
    if ((this->mountMotion.GoToIsActiveInRA == false) && (this->mountMotion.GoToIsActiveInDecl == false)) {
        isInGoTo = false;
    } else {
        isInGoTo = true;
    } // checks whether GoTo was deactivated

    if (isEast != g_AllData->getMFlipParams(1)) {
        // deactivate the "isEast" checkbox so that user input is no longer feasible
        ui->cbMountIsEast->setDisabled(true);
        disconnect(ui->cbMountIsEast, SIGNAL(stateChanged(int)), 0, 0);
    }
    if (g_AllData->getMFlipParams(1) == true) {
        ui->cbMountIsEast->setChecked(true);
    } else {
        ui->cbMountIsEast->setChecked(false);
    }

    if ((wasInGoTo == true) && (isInGoTo == false)) { // slew has stopped
        this->terminateGoTo(false);
    }

    if (this->ccdMainCameraIsAcquiring == true) {
        this->psImageAcquisionTimeRemaining = round(ui->sbPSExposureTime->value() - this->elapsedPS->elapsed()*0.001);
        if (this->psImageAcquisionTimeRemaining < 0) {
            this->psImageAcquisionTimeRemaining = 0;
            ui->cbPSImageInTransfer->setChecked(true);
        }
        ui->lcdETAOfPSImage->display((int)(this->psImageAcquisionTimeRemaining));
    }
}
//------------------------------------------------------------------------
// a routine that computes a string out of decimal coordinates for RA and decl

QString* MainWindow::generateCoordinateString(float coord, bool isRA) {
    int RAHrs, RAMin, RASec; // a few little helpers for displaying RA
    int declDeg, declMin, declSec,sign = 1; // same for Declination
    QString *helper;
    double currRA, remainder,RAInHours, currDecl;

    helper = new QString();
    this->coordString->clear();
    if (isRA == true) {
        currRA = coord;
        RAInHours = currRA/360.0*24.0;
        if (RAInHours > 24) {
            RAInHours -= 24;
        }
        RAHrs = floor(RAInHours);
        RAMin = floor((RAInHours - RAHrs)*60.0);
        remainder = ((RAInHours - RAHrs)*60.0) - RAMin;
        RASec = round(remainder*60.0);
        if (RAHrs < 10) {
            this->coordString->append("0");
        }
        helper->setNum(RAHrs);
        this->coordString->append(helper);
        helper->clear();
        this->coordString->append("h ");
        if (RAMin < 10) {
            this->coordString->append("0");
        }
        helper->setNum(RAMin);
        this->coordString->append(helper);
        helper->clear();
        this->coordString->append("m ");
        if (RASec < 10) {
            this->coordString->append("0");
        }
        helper->setNum(RASec);
        this->coordString->append(helper);
        helper->clear();
        this->coordString->append("s");
    } else {
        currDecl = coord;
        if (currDecl < 0) {
            sign = -1;
        } else {
            sign = 1;
        }
        declDeg=(int)(sign*floor(fabs(currDecl)));
        remainder = fabs(currDecl-((double)declDeg));
        declMin=(int)(floor(remainder*60.0));
        remainder = remainder*60.0-declMin;
        declSec=round(remainder*60.0);
        if (sign == 1) {
            this->coordString->insert(0,'+');
        } else {
            this->coordString->insert(0,'-');
        }

        if ((abs(declDeg)) < 10) {
            this->coordString->insert(1,'0');
        }
        helper->setNum(abs(declDeg));
        this->coordString->append(helper);
        helper->clear();
        this->coordString->append("° ");
        if (declMin < 10) {
            this->coordString->append("0");
        }
        helper->setNum(declMin);
        this->coordString->append(helper);
        helper->clear();
        this->coordString->append("' ");
        if (declSec < 10) {
            this->coordString->append("0");
        }
        helper->setNum(declSec);
        this->coordString->append(helper);
        this->coordString->append("''");
    }
    delete helper;
    return this->coordString;
}


//------------------------------------------------------------------------
// routine for handling date and time; also computes julian day and local sidereal
// time
void MainWindow::updateTimeAndDate(void) {
    double secSinceMidnight, lstX, lst;

    ui->leTime->setText(this->UTTime->currentTime().toString());
    ui->leDate->setText(this->UTDate->currentDate().toString("dd/MM/yyyy"));
    this->julianDay = this->UTDate->toJulianDay()-0.5;
    ui->teJulianDay->setText(QString::number(((long)(this->julianDay))));
    secSinceMidnight=UTTime->currentTime().hour()*3600.0+UTTime->currentTime().minute()*60.0+UTTime->currentTime().second()+UTTime->currentTime().msec()/1000.0;
    lstX=(this->julianDay-2451545.0)/36525.0;
    lst=6.697374558 + 2400.051336*lstX + 0.000025862*lstX*lstX + 1.00273791*(secSinceMidnight/3600.0) + g_AllData->getSiteCoords(1)/15.0;
    lst=lst-(int(lst/24))*24.0;
    ui->teLSTime->setText(QString::number(lst,'f',5));
    g_AllData->setLocalSTime(lst); // store the sidereal time also in the global data class ...
}

//---------------------------------------------------------------------
// just a little helper to wait for a bunch of milliseconds
void MainWindow::waitForNMSecs(int msecs) {
    QElapsedTimer *qelap;

    qelap = new QElapsedTimer();
    qelap->start();
    do {
      QCoreApplication::processEvents(QEventLoop::AllEvents, msecs);
    } while (qelap->elapsed() < msecs);
    delete qelap;
}
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// routines for basic stepper operation
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// the most important routine - is compensates for earth motion, sets all flags and disables all GUI elements that can interfere
void MainWindow::startRATracking(void) {
    this->isInParking = false; // true if a parking motion was carried out before ...
    ui->rbCorrSpeed->setEnabled(true);
    ui->rbMoveSpeed->setEnabled(true);
    if (ui->rbMoveSpeed->isChecked() == false) {
        ui->sbMoveSpeed->setEnabled(true);
    }
    this->StepperDriveRA->stopDrive();
    this->mountMotion.RATrackingIsOn = true;
    ui->pbStartTracking->setEnabled(0);
    ui->pbStopTracking->setEnabled(1);
    this->raState = guideTrack;
    this->StepperDriveRA->changeMicroSteps(g_AllData->getMicroSteppingRatio(0));
    this->mountMotion.RAtrackingElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
    this->StepperDriveRA->startTracking();
    this->setControlsForRATracking(false);
    g_AllData->setTrackingMode(true);
    qDebug() << "RA is tracking...";
}

//------------------------------------------------------------------
// stops earthtracking. has to be called prior to all other commands issued to the RA-stepper
void MainWindow::stopRATracking(void) {
    this->setControlsForRATracking(true);
    ui->pbStartTracking->setEnabled(1);
    ui->pbStopTracking->setEnabled(0);
    this->StepperDriveRA->stopDrive();
    this->mountMotion.RATrackingIsOn = false;
    g_AllData->setTrackingMode(false);
}

//------------------------------------------------------------------
// synchronizes the mount to given coordinates and sets the monotonic timer to zero
void MainWindow::syncMount(void) {
    if (this->StepperDriveRA->getStopped() == false) { // stop tracking
        this->stopRATracking();
    }
    if (this->mountMotion.DeclDriveIsMoving == true) {
        this->mountMotion.DeclDriveIsMoving=false;
        this->StepperDriveDecl->stopDrive();
    } // stop the declination drive as well ...
    g_AllData->setSyncPosition(this->ra, this->decl);
    // convey right ascension and declination to the global parameters;
    // a microtimer starts ...
    this->startRATracking(); // start tracking again
    ui->pbGoTo->setEnabled(true); // enable GOTO as we now have a reference position
    ui->gbScopeParking->setEnabled(true); // enable the park position as the scope is now synced
    ui->pbGoToPark->setEnabled(true);
    ui->pbStorePark->setEnabled(true);
    if (g_AllData->getMFlipParams(0) == true) { //check if flip is enabled
        ui->cbMountIsEast->setEnabled(true);
        connect(ui->cbMountIsEast, SIGNAL(stateChanged(int)), this, SLOT(mountIsEast())); // act whether the mount is set to east-west
    } // and re-enable the left of pier/right of pier thing
}

//-----------------------------------------------------------------
// same as above, but not called as a slot
void MainWindow::syncMountFromGoTo(void) {
    if (this->StepperDriveRA->getStopped() == false) { // stop tracking
        this->stopRATracking();
    }
    if (this->mountMotion.DeclDriveIsMoving == true) {
        this->mountMotion.DeclDriveIsMoving=false;
        this->StepperDriveDecl->stopDrive();
    } // stop the declination drive as well ...
    g_AllData->setSyncPosition(this->ra, this->decl);
    // convey right ascension and declination to the global parameters;
    // a microtimer starts ...
    this->startRATracking(); // start tracking again
    ui->pbGoTo->setEnabled(true); // enable GOTO as we now have a reference position
    ui->gbScopeParking->setEnabled(true); // enable the park position as the scope is now synced
    ui->pbGoToPark->setEnabled(true);
    ui->pbStorePark->setEnabled(true);
}

//------------------------------------------------------------------
// synchronizes the mount to coordinates provided and sets the monotonic timer to zero
void MainWindow::syncMount(float lra, float lde, bool isEmergencyStop) {
    if (this->StepperDriveRA->getStopped() == false) { // stop tracking
        this->stopRATracking();
    }
    if (this->mountMotion.DeclDriveIsMoving == true) {
        this->mountMotion.DeclDriveIsMoving=false;
        this->StepperDriveDecl->stopDrive();
    } // stop the declination drive as well ...
    g_AllData->setSyncPosition(lra, lde);
    // convey right ascension and declination to the global parameters;
    // a microtimer starts ...
    if (isEmergencyStop == false) {
        this->startRATracking(); // start tracking again
    }
    ui->pbGoTo->setEnabled(true); // enable GOTO as we now have a reference position
    ui->gbScopeParking->setEnabled(true); // enable the park position as the scope is now synced
    ui->pbGoToPark->setEnabled(true);
    ui->pbStorePark->setEnabled(true);
}
//---------------------------------------------------------------------
// that one handles GOTO-commands. it leaves when the destination is reached ...
void MainWindow::startGoToObject(void) {
    double travelRA, travelDecl, absShortRATravel, speedFactorRA, speedFactorDecl,TRamp, SRamp,
           SAtFullSpeed, TAtFullSpeed, earthTravelDuringGOTOinMSteps, mstepRatio,
           convertDegreesToMicrostepsDecl,convertDegreesToMicrostepsRA, targetHA, localHA; // variables for assessing travel time and so on
  //  qint64 timestampGOTOStarted; // various time stamps
    qint64 timeEstimatedInRAInMS = 0; // estimate for travel time in RA [ms]
    qint64 timeEstimatedInDeclInMS = 0; // estimate for travel time in Decl [ms]
    long int RASteps, DeclSteps; // microsteps for travel plus a correction measure
    int timeForProcessingEventQueue = 100; // should be the same as the time for the event queue given in this->timer
    short flipResult = 0;

    g_AllData->setCelestialSpeed(0); // make sure that the drive speed is sidereal
    ui->rbSiderealSpeed->setChecked(true);
    ui->pbGoTo->setEnabled(false); // disable pushbutton for GOTO
    this->setControlsForGoto(false); // set some controls on disabled
    ui->pbStartTracking->setEnabled(false); // tracking button is disabled
    if (this->ccdGuiderCameraIsAcquiring == true) { // slewing and transfer of FITS images at the same time cause erratic behaviour,
        this->stopCCDAcquisition();           // the guiding camera acquisition is terminated if active
    }
    this->syncMount(g_AllData->getActualScopePosition(2), g_AllData->getActualScopePosition(1),false);
    // make a sync to the topicalposition

    travelRA=((g_AllData->getActualScopePosition(0))+g_AllData->getCelestialSpeed()*g_AllData->getTimeSinceLastSync()/1000.0)-this->ra;
    if (fabs(travelRA) > 180) {
        absShortRATravel = 360.0 - fabs(travelRA);
        if (travelRA > 0) {
            travelRA = -absShortRATravel;
        } else {
            travelRA = absShortRATravel;
        }
    } // determine the shorter travel path
    travelDecl=this->decl-g_AllData->getActualScopePosition(1); // travel in both axes based on current position

    localHA = (g_AllData->getLocalSTime()*15 - g_AllData->getActualScopePosition(2));
    while (localHA < 0) {
        localHA += 360;
    }
    while (localHA > 360) {
        localHA -= 360;
    }
    targetHA = localHA+travelRA;
    if (targetHA > 360){
        targetHA -= 360;
    }
    if (targetHA < 360) {
        targetHA += 360;
    }// calculated the estimated hour angle at target position

    flipResult = this->checkForFlip(g_AllData->getMFlipParams(1),localHA,targetHA, g_AllData->getActualScopePosition(1), this->decl);
    if (flipResult != 0) {
        if (flipResult == -1) {
            travelRA = -(180 - travelRA);
        } else {
            travelRA= 180 + travelRA;
        }
        travelDecl=(90 - g_AllData->getActualScopePosition(1))*2-travelDecl;
    } // modified travel for meridian flip if needed


    this->targetRA = this->ra;
    this->targetDecl = this->decl; // destination as given by LX200 or the menu of TSC
    if (travelRA < 0) {
        this->mountMotion.RADriveDirection = -1;
    } else {
        this->mountMotion.RADriveDirection = 1;
    } // determine direction in RA
    if (travelDecl < 0) {
        this->mountMotion.DeclDriveDirection = -1*g_AllData->getMFlipDecSign();
    } else {
        this->mountMotion.DeclDriveDirection = 1*g_AllData->getMFlipDecSign();
    } // determine direction in declination

    this->raState = slew;
    this->deState = slew;
    this->StepperDriveRA->changeMicroSteps(g_AllData->getMicroSteppingRatio(2));
    this->StepperDriveDecl->changeMicroSteps(g_AllData->getMicroSteppingRatio(2));
    mstepRatio = g_AllData->getMicroSteppingRatio(2)/((double)(g_AllData->getMicroSteppingRatio(0)));
    speedFactorDecl=ui->sbGoToSpeed->value()*mstepRatio;
    speedFactorRA=ui->sbGoToSpeed->value()*mstepRatio;    // set the drive speed to GOTO speed according to spinbox in GUI
    convertDegreesToMicrostepsDecl=1.0/g_AllData->getGearData(7)*g_AllData->getMicroSteppingRatio(2)*
            g_AllData->getGearData(4)*g_AllData->getGearData(5)*g_AllData->getGearData(6);
    DeclSteps=round(fabs(travelDecl)*convertDegreesToMicrostepsDecl); // determine the number of microsteps necessary to reach target. direction is already given and unimportant here ...
    convertDegreesToMicrostepsRA=1.0/g_AllData->getGearData(3)*g_AllData->getMicroSteppingRatio(2)*
            g_AllData->getGearData(0)*g_AllData->getGearData(1)*g_AllData->getGearData(2);
    RASteps=round(fabs(travelRA)*convertDegreesToMicrostepsRA); // determine the number of microsteps necessary to reach target. direction is already given and unimportant here ...
    // ------------------------------- computed gross distance for ra and decl
    TRamp = (this->StepperDriveDecl->getKineticsFromController(3)*(speedFactorDecl))/this->StepperDriveDecl->getKineticsFromController(2);// time needed until drive reaches full speed - vel/acc ...
    SRamp = 0.5*this->StepperDriveDecl->getKineticsFromController(2)*TRamp*TRamp; // travel in microsteps until full speed is reached
    SAtFullSpeed = fabs(DeclSteps)-2.0*SRamp; // travel after acceleration and before de-acceleration
    if (SAtFullSpeed < 0) {
        TAtFullSpeed=sqrt(DeclSteps/this->StepperDriveDecl->getKineticsFromController(2));// if the travel is so short that full speed cannot be reached: consider a ramp that stops at the end of travel
        timeEstimatedInDeclInMS = (TAtFullSpeed)*1000+timeForProcessingEventQueue; // time in microseconds estimated for Declination-Travel
    } else {
        TAtFullSpeed = SAtFullSpeed/(this->StepperDriveDecl->getKineticsFromController(3)*speedFactorDecl);
        timeEstimatedInDeclInMS = (TAtFullSpeed+2.0*TRamp)*1000+timeForProcessingEventQueue; // time in microseconds estimated for Declination-Travel
    }
    // Now repeat that computation for the RA drive
    TRamp = (this->StepperDriveRA->getKineticsFromController(3)*(speedFactorRA))/this->StepperDriveRA->getKineticsFromController(2);
    SRamp = 0.5*this->StepperDriveRA->getKineticsFromController(2)*TRamp*TRamp;
    SAtFullSpeed = fabs(RASteps)-2.0*SRamp;
    if (SAtFullSpeed < 0) {
        TAtFullSpeed=sqrt(RASteps/this->StepperDriveRA->getKineticsFromController(2));
        timeEstimatedInRAInMS = (TAtFullSpeed)*1000+timeForProcessingEventQueue;
    } else {
        TAtFullSpeed = SAtFullSpeed/(this->StepperDriveRA->getKineticsFromController(3)*speedFactorRA);
        timeEstimatedInRAInMS = (TAtFullSpeed+2.0*TRamp)*1000+timeForProcessingEventQueue;
    }

    earthTravelDuringGOTOinMSteps=(g_AllData->getCelestialSpeed()*((double)timeEstimatedInRAInMS)/1000.0)*
            convertDegreesToMicrostepsRA; // determine the addition of earth travel in sideral time into account

        // compensate for earth travel in fwd or bwd direction
    if (this->mountMotion.RADriveDirection == 1) {
        RASteps=RASteps+earthTravelDuringGOTOinMSteps;
    } else {
        RASteps=RASteps-earthTravelDuringGOTOinMSteps;
    }
    timeEstimatedInRAInMS = RASteps/((double)this->StepperDriveRA->getKineticsFromController(3)*(speedFactorRA))*1000; // correct RA travel time for this additional motion
    // finished travel time considerations ...
    this->approximateGOTOSpeedRA=RASteps/(timeEstimatedInRAInMS/1000.0); // for LX 200 display, a mean speed during GOTO not taking ramps into account is computed; it is shortened to avoid overshooting (in graphical display)
    this->approximateGOTOSpeedDecl=DeclSteps/(timeEstimatedInDeclInMS/1000.0); // same as above
    if (timeEstimatedInRAInMS > timeEstimatedInDeclInMS) {
        gotoETA = timeEstimatedInRAInMS;
    } else {
        gotoETA = timeEstimatedInDeclInMS;
    }
    ui->lcdGotoTime->display(round(gotoETA/1000.0)); // determined the estimated duration of the GoTo - Process and display it in the GUI. it is reduced in the event queue
    // let the games begin ... GOTO is ready to start ...
    this->terminateAllMotion(); // stop the drives
    this->elapsedGoToTime->start(); // a second timer in the class to measure the time elapsed during goto - needed for updates in the event queue
    this->StepperDriveRA->travelForNSteps(RASteps,this->mountMotion.RADriveDirection,round(speedFactorRA/mstepRatio),false);
    this->mountMotion.GoToIsActiveInRA=true;
    this->mountMotion.RAGoToElapsedTimeInMS=g_AllData->getTimeSinceLastSync();
    //timestampGOTOStarted = g_AllData->getTimeSinceLastSync();
    ui->pbStartTracking->setEnabled(false);
    this->StepperDriveDecl->travelForNSteps(DeclSteps,this->mountMotion.DeclDriveDirection*g_AllData->getMFlipDecSign(),round(speedFactorDecl/mstepRatio),0);
    this->mountMotion.GoToIsActiveInDecl=true;
    this->mountMotion.DeclGoToElapsedTimeInMS=g_AllData->getTimeSinceLastSync(); // now, all drives are started and timestamps were taken
}

//------------------------------------------------------------------
// this routine handles finishing a GoTo
void MainWindow::terminateGoTo(bool calledAsEmergencyStop) {
    qDebug() << "Parking state: " << this->isInParking;
    this->ra=this->targetRA;
    this->decl=this->targetDecl;

    ui->lcdGotoTime->display(0); // set the LCD counter to zero again
    this->setControlsForGoto(true);
    ui->pbStopTracking->setDisabled(false);
    this->setControlsForRATravel(true); // set GUI back in base state
    this->mountMotion.GoToIsActiveInRA=false;
    this->mountMotion.GoToIsActiveInDecl=false; // just to make sure - slew has ENDED here ...
    this->StepperDriveDecl->changeMicroSteps(g_AllData->getMicroSteppingRatio(0));
    if (this->isInParking == false) {
        if (calledAsEmergencyStop == false) {
            this->syncMountFromGoTo(); // sync the mount to desired position
        } else {
            this->syncMount(g_AllData->getActualScopePosition(2), g_AllData->getActualScopePosition(1),true);
            // in an emergency stop, sync the mount to actual position
        }
        this->setControlsForRATracking(false);

    } else {
        this->setControlsForRATracking(true);
        ui->pbStartTracking->setDisabled(false);
        this->stopRATracking();
        this->isInParking = false;
    }
    this->deState = guideTrack;
    this->StepperDriveDecl->setInitialParamsAndComputeBaseSpeed((double)ui->sbAMaxDecl_AMIS->value(),
                                                                ((double)(round(ui->sbCurrMaxDecl_AMIS->value()))));
    this->meridianFlipDisabledForPolarParking = false; // if this was a flip to the north pole, it is done now ...
}

//------------------------------------------------------------------
// a routine that checks whether a meridian flip is necessary; returns 0 for no flip, 1, for a flip to west, -1 for a flip to east
short MainWindow::checkForFlip(bool isEast, float ha, float gha, float dec, float gDec) {
    short doFlip = 0;
    short oQuad = 1, tQuad = 1;
    short maxDecl;

    if (gha > 360) {
        gha -=360;
    }
    if (gha < 0) {
        gha +=360;
    }

    if (this->meridianFlipDisabledForPolarParking == true) {
        return 0; // no flip for going to the north pole
    }
    if (g_AllData->getMFlipParams(0) == false) {
        return 0; // if no meridian flip is needed by the mount ... don't do it
    } else {
        if ((ha >= 0) && (ha < 90)) {
            oQuad = 1;
        }
        if ((ha >= 90) && (ha < 180)) {
            oQuad = 2;
        }
        if ((ha >= 180) && (ha < 270)) {
            oQuad = 3;
        }
        if ((ha >= 270) && (ha < 360)) {
            oQuad = 4;
        }

        if ((gha >= 0) && (gha < 90)) {
            tQuad = 1;
        }
        if ((gha >= 90) && (gha < 180)) {
            tQuad = 2;
        }
        if ((gha >= 180) && (gha < 270)) {
            tQuad = 3;
        }
        if ((gha >= (270)) && (gha < 360)) {
            tQuad = 4;
        }
        if (isEast == true) {
            switch (oQuad) {
                case 1:
                    switch (tQuad) {
                    case 1: doFlip = 0; break;
                    case 2: doFlip = 0; break;
                    case 3: doFlip = 1; break;
                    case 4: doFlip = 1; break;
                    }
                    break;
                case 2:
                    switch (tQuad) {
                    case 1: doFlip = 0; break;
                    case 2: doFlip = 0; break;
                    case 3: doFlip = -1; break;
                    case 4: doFlip = -1; break;
                    }
                    break;
                case 3:
                    switch (tQuad) {
                    case 1: doFlip = -1; break;
                    case 2: doFlip = -1; break;
                    case 3: doFlip = -1; break;
                    case 4: doFlip = -1; break;
                    }
                    break;
                case 4:
                switch (tQuad) {
                    case 1: doFlip = 1; break;
                    case 2: doFlip = 1; break;
                    case 3: doFlip = 1; break;
                    case 4: doFlip = 1; break;
                    }
                    break;
            }
        } else {
            switch (oQuad) {
                case 1:
                    switch (tQuad) {
                        case 1: doFlip = -1; break;
                        case 2: doFlip = -1; break;
                        case 3: doFlip = -1; break;
                        case 4: doFlip = -1; break;
                    }
                break;
                case 2:
                    switch (tQuad) {
                        case 1: doFlip = -1; break;
                        case 2: doFlip = -1; break;
                        case 3: doFlip = -1; break;
                        case 4: doFlip = -1; break;
                    }
                break;
                case 3:
                    switch (tQuad) {
                        case 1: doFlip = 1; break;
                        case 2: doFlip = 1; break;
                        case 3: doFlip = 0; break;
                        case 4: doFlip = 0; break;
                    }
                break;
                case 4:
                    switch (tQuad) {
                        case 1: doFlip = -1; break;
                        case 2: doFlip = -1; break;
                        case 3: doFlip = 0; break;
                        case 4: doFlip = 0; break;
                    }
                break;
            }
        }
    }
    qDebug() << "Flip result: ---------------------------";
    qDebug() << "Left of Pier: "<< isEast << "Origin: " << oQuad << ", Target: " << tQuad << ", Flip Result: " << doFlip;
    qDebug() << "----------------------------------------";
    maxDecl = g_AllData->getMaxDeclForNoFlip();
    if ((dec < maxDecl) && (gDec < maxDecl)) {
        doFlip = 0;
        qDebug() << "Disable Meridian flip because of low declination...";
    } // check if you are below the maximum declination for not carrying out a flip

    return doFlip;
}

//------------------------------------------------------------------
// this routine computes a fixed parking position from the current scope position
void MainWindow::determineParkingPosition(void) {
    float parkHA;
    float parkDecl;

    parkHA = (g_AllData->getLocalSTime()*15 - g_AllData->getActualScopePosition(2));
    while (parkHA < 0) {
        parkHA += 360;
    }
    while (parkHA > 360) {
        parkHA -= 360;
    }
    parkDecl = g_AllData->getActualScopePosition(1); // got current position
    ui->lcdHAPark->display(parkHA);
    ui->lcdDecPark->display(parkDecl);
    g_AllData->setParkingPosition(parkHA, parkDecl);
    g_AllData->storeGlobalData(); // ... and stored it to the preferences file
}

//------------------------------------------------------------------
// sets a few selected park positions
void MainWindow::presetParkingPositionPolaris(void) {
    float parkHA = 0, parkDecl = 0;

    parkDecl = 89.9;
    parkHA = 18.0*15;
    ui->lcdHAPark->display(parkHA);
    ui->lcdDecPark->display(parkDecl);
    g_AllData->setParkingPosition(parkHA, parkDecl);
    g_AllData->storeGlobalData(); // ... and stored it to the preferences file
}

//------------------------------------------------------------------
// sets a few selected park positions
void MainWindow::presetParkingPositionSouthH(void) {
    float parkHA = 0, parkDecl = 0;

    parkDecl = -(90-g_AllData->getSiteCoords(0));
    parkHA = 0.0;
    ui->lcdHAPark->display(parkHA);
    ui->lcdDecPark->display(parkDecl);
    g_AllData->setParkingPosition(parkHA, parkDecl);
    g_AllData->storeGlobalData(); // ... and stored it to the preferences file
}

//------------------------------------------------------------------
// this routine parks the telescope at the stored parking position
void MainWindow::gotoParkPosition(void) {
    this->ra   = g_AllData->getLocalSTime()*15 - g_AllData->getParkingPosition(0);
    this->decl = g_AllData->getParkingPosition(1);
    if (fabs(this->decl) > 85) {
        this->meridianFlipDisabledForPolarParking = true;
    }
    this->startGoToObject();
    this->isInParking = true;
}

//------------------------------------------------------------------
// this routine syncs the telescope at the stored parking position
void MainWindow::syncParkPosition(void) {
    this->ra   = g_AllData->getLocalSTime()*15 - g_AllData->getParkingPosition(0);
    this->decl = g_AllData->getParkingPosition(1);
    this->syncMount(); // sync the mount
}

//------------------------------------------------------------------
// handles shutdown of the program
void MainWindow::shutDownProgram() {
    bool isEast;

    qDebug() << "Shutting down TSC...";
    qDebug() << "Shutting down drives...";
    this->StepperDriveRA->shutDownDrive();
    this->StepperDriveDecl->shutDownDrive();
    if (this->auxBoardIsAvailable == true) {
        emergencyStopAuxDrives();
    }
    system("sudo timedatectl set-ntp 1");
    qDebug() << "Storing position...";
    isEast = ui->cbMountIsEast->isChecked();
    std::ofstream outfile(".GEMState.tsl");
    std::string ostr;
    if (isEast == true) {
        ostr.append("1");
    } else {
        ostr.append("0");
    }
    outfile << ostr.data();
    outfile.close(); // close the file
    ostr.clear();  // save the state of the GEM in a separate file at shutdown
    qDebug() << "Shutting down camera...";
    this->ccdGuiderCameraIsAcquiring=false;
    this->waitForNMSecs((ui->sbExposureTime->value())*1000);
    camera_client->sayGoodbyeToINDIServer();
    this->waitForNMSecs(500);
    qDebug() << "Freeing memory ...";
    delete currentRAString;
    delete currentDeclString;
    delete currentHAString;
    delete coordString;
    delete textEntry;
    delete wcsInfoOutput;
    delete camImg;
    delete mainCamImg;
    delete guideStarPrev;
    delete guideCamDriverName;
    delete mainCamDriverName;
    qDebug() << "Freeing SPI ...";
    delete spiDrOnChan0;
    delete spiDrOnChan1;
    qDebug() << "Freeing LX200...";
    delete lx200Comm;
    delete LXServer;
    delete LXSocket;
    delete LXServerAddress;
    delete tcpLXdata;
    delete HBSocket;
    delete HBServer;
    delete HBServerAddress;
    delete tcpHBData;
    delete lx200SerialPort;
    delete lx200SerialData;
    qDebug() << "Freeing timers ...";
    delete UTDate;
    delete UTTime;
    delete timer;
    delete checkDriveTimer;
    delete st4State.raCorrTime;
    delete st4State.deCorrTime;
    delete elapsedGoToTime;
    delete elapsedPS;
    delete g_AllData;
    qDebug() << "Closing USB bus ...";
    delete amisInterface;
    qDebug() << "Killing AMetry Process...";
    delete astroMetryProcess;
    qDebug() << "Freeing camera client ...";
    delete camera_client;
    qDebug() << "Goodbye!";
    exit(0);
}

//---------------------------------------------------------------------
// called to check whether the error flag on the AMIS drivers is up ...
void MainWindow::getDriveError(void) {
/*    bool isError;

    isError = this->StepperDriveRA->getErrorFromDriver();
    if (isError == true) {
        ui->cbErrRA->setChecked(true);
        this->StepperDriveRA->hwResetDriver();
        this->waitForNMSecs(500);
    }
    isError = this->StepperDriveDecl->getErrorFromDriver();
    if (isError == true) {
        ui->cbErrDecl->setChecked(true);
        this->StepperDriveDecl->hwResetDriver();
        this->waitForNMSecs(500);

    }*/
}

//---------------------------------------------------------------------
// soft stop for drives
void MainWindow::terminateAllMotion(void) {
    if (this->mountMotion.RADriveIsMoving == true) {
        this->mountMotion.RADriveIsMoving=false;
        this->StepperDriveRA->stopDrive();
    }
    if (this->mountMotion.DeclDriveIsMoving == true) {
        this->mountMotion.DeclDriveIsMoving=false;
        this->StepperDriveDecl->stopDrive();
    }
    if (this->mountMotion.RATrackingIsOn == true) {
        this->stopRATracking();
    }
}

//---------------------------------------------------------------------
// emergency stop of all motion
void MainWindow::emergencyStop(void) {
    this->mountMotion.emergencyStopTriggered=true;
    this->StepperDriveRA->stopDrive();
    this->StepperDriveDecl->stopDrive();
    if ((this->mountMotion.GoToIsActiveInRA == true) || (this->mountMotion.GoToIsActiveInDecl == true)) {
        this->terminateGoTo(true);
    }
    this->mountMotion.RATrackingIsOn = false;
    this->mountMotion.RADriveIsMoving = false;
    this->mountMotion.DeclDriveIsMoving = false;
    this->mountMotion.GoToIsActiveInRA = false;
    this->mountMotion.GoToIsActiveInDecl = false;
    ui->pbStartTracking->setEnabled(true);
    ui->pbStopTracking->setEnabled(false);
    this->setControlsForGoto(true);
    ui->lcdGotoTime->display(0);
    ui->pbGoTo->setEnabled(true);
    ui->ctrlTab->setEnabled(true);
}
//---------------------------------------------------------------------
// slot for changing drive speeds
void MainWindow::setCorrectionSpeed(void) {
    this->mountMotion.RASpeedFactor = 1;
    this->mountMotion.DeclSpeedFactor = 1;
    ui->sbMoveSpeed->setEnabled(true);
}

//---------------------------------------------------------------------
// slot for changing drive speeds
void MainWindow::setMoveSpeed(void) {
    this->mountMotion.RASpeedFactor = ui->sbMoveSpeed->value();
    this->mountMotion.DeclSpeedFactor = ui->sbMoveSpeed->value();
    ui->sbMoveSpeed->setEnabled(false);
}
//---------------------------------------------------------------------
// slot for changing drive speeds
void MainWindow::changeMoveSpeed(void) {
    this->mountMotion.RASpeedFactor = ui->sbMoveSpeed->value();
    this->mountMotion.DeclSpeedFactor = ui->sbMoveSpeed->value();
    ui->rbMoveSpeed->setChecked(true);
}

//---------------------------------------------------------------------
// invert direction for RA if on souther hermisphere - is a slot
void MainWindow::invertRADirection(void) {
    if (ui->cbIsOnNorthernHemisphere->isChecked() == true) {
        this->RAdriveDirectionForNorthernHemisphere = 1;
    } else {
        this->RAdriveDirectionForNorthernHemisphere = -1;
    }
    this->StepperDriveRA->setRADirection(this->RAdriveDirectionForNorthernHemisphere);
}

//------------------------------------------------------------------
// convey acceleration to the stepper class from the GUI
void MainWindow::setMaxStepperAccRA(void) {
    double val = 2500;
    bool trackingWasOn;

    trackingWasOn = this->mountMotion.RATrackingIsOn;
    if (this->mountMotion.RATrackingIsOn == true) {
        this->stopRATracking();
    }
    val = (double)(ui->sbAMaxRA_AMIS->value());
    this->StepperDriveRA->setStepperParams(val, 1);
    if (trackingWasOn == true) {
        this->startRATracking();
    }
    g_AllData->setDriveParams(0,1,val);
}

//------------------------------------------------------------------
// convey acceleration to the stepper class from the GUI
void MainWindow::setMaxStepperAccDecl(void) {
    double val = 2500;

    val = (double)(ui->sbAMaxDecl_AMIS->value());
    this->StepperDriveDecl->setStepperParams(val, 1);
    g_AllData->setDriveParams(1,1,val);
}

//------------------------------------------------------------------
// convey current to the stepper class from the GUI
void MainWindow::setMaxStepperCurrentRA(void) {
    double val = 0.1;
    bool trackingWasOn;

    trackingWasOn = this->mountMotion.RATrackingIsOn;
    if (this->mountMotion.RATrackingIsOn == true) {
        this->stopRATracking();
    }
    val = ui->sbCurrMaxRA_AMIS->value();
    this->StepperDriveRA->setStepperParams(val, 3);
    if (trackingWasOn == true) {
        this->startRATracking();
    }
    g_AllData->setDriveParams(0,2,val);
}
//------------------------------------------------------------------
// convey current to the stepper class from the GUI
void MainWindow::setMaxStepperCurrentDecl(void) {
    double val=0.1;

    val = ui->sbCurrMaxDecl_AMIS->value();
    this->StepperDriveDecl->setStepperParams(val, 3);
    g_AllData->setDriveParams(1,2,val);
}

//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
// routines for communication with the INDI server
//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
// read the address and port from the GUI and start connect the guide camera to the INDI server
void MainWindow::setINDISAddrAndPort(void) {
    QString saddr;
    int sport;
    bool isServerUp = 0;
    QMessageBox noCamBoxMsg;

    if ((this->cam1Selected == false) && (this->cam2Selected == false)) {
            return;
    }
    ui->pbConnectToServer->setEnabled(false);
    saddr=ui->leINDIServer->text();
    sport=ui->sbINDIPort->value();
    isServerUp = camera_client->setINDIServer(saddr,sport);
    qDebug() << "INDI Server is set...";
    g_AllData->setINDIState(isServerUp,false);
    // set a global flag on the server state
    if (isServerUp==true) {
        this->waitForNMSecs(500);
        QCoreApplication::processEvents(QEventLoop::AllEvents,2000);   // process events before sleeping for a second
        this->waitForNMSecs(1000);
        isServerUp = camera_client->probeForCCD(0); // at least one camera should be connected
        if (isServerUp == true) {
            ui->cbIndiIsUp->setChecked(true);
            ui->pbKillINDIServer->setEnabled(false);
            ui->cbStoreGuideCamImgs->setEnabled(true);
            ui->leCam1Name->setText(camera_client->getINDIName(0)->toLatin1());
            ui->leCam2Name->setText(camera_client->getINDIName(1)->toLatin1());
            ui->pbDisconnectFromServer->setEnabled(true);
            ui->rbMainCCD->setEnabled(true);
            ui->rbGuiderCCD->setEnabled(true);
        } else {
            ui->pbConnectToServer->setEnabled(true);
            ui->pbExpose->setEnabled(false);
            ui->cbIndiIsUp->setChecked(false);
            ui->cbStoreGuideCamImgs->setEnabled(false);
            ui->rbMainCCD->setEnabled(true);
            ui->rbGuiderCCD->setEnabled(true);
            noCamBoxMsg.setWindowTitle("Critical INDI error");
            noCamBoxMsg.setText("Camera not available - is it connected?");
            noCamBoxMsg.exec();
            camera_client->sayGoodbyeToINDIServer();
            this->disconnectFromINDIServer();
            this->waitForNMSecs(1000);
        }
    } else {
          ui->pbConnectToServer->setEnabled(true);
    }
}

//------------------------------------------------------------------
// after selection of the guidecam, the proper parameters are retrieved here
void MainWindow::selectCameraTypes(void){
    if (((this->cam1Selected == true) && (this->cam2Selected == false)) || ((this->cam1Selected == false) && (this->cam2Selected == true))) { // only one active INDI camera, it can be main or guider
        if (ui->rbMainCCD->isChecked() == true) {
            this->mainCamSelected = true;
            this->guiderCamSelected = false;
        } else {
            this->guiderCamSelected = true;
            this->mainCamSelected = false;
        }
    }

    if ((this->cam1Selected == true) && (this->cam2Selected == true)) {
        this->mainCamSelected = true;
        this->guiderCamSelected = true;
    }
    g_AllData->setINDIState(mainCamSelected, true);
    g_AllData->setINDIState(guiderCamSelected, false); // storing which cameras are now connected in the global struct
    if (ui->rbMainCCD->isChecked() == true) {
        camera_client->setIdxOfCCDs(0,true);
        camera_client->setIdxOfCCDs(1,false);
        ui->pbTakeImagePS->setEnabled(true);
        ui->gbCCDParamsMainCCD->setEnabled(true);
        camera_client->loadINDIConfigFile(true);
        this->getCCDParameters(true);
        if (this->guiderCamSelected == true) {
            camera_client->loadINDIConfigFile(false);
            this->getCCDParameters(false);
            ui->gbCCDParams->setEnabled(true);
            ui->pbExpose->setEnabled(true);
        }
    }
    if (ui->rbGuiderCCD->isChecked() == true) {
        camera_client->setIdxOfCCDs(0, false);
        camera_client->setIdxOfCCDs(1,true);
        ui->gbCCDParams->setEnabled(true);
        ui->pbExpose->setEnabled(true);
        camera_client->loadINDIConfigFile(false);
        this->getCCDParameters(false);
        if (this->mainCamSelected == true) {
            camera_client->loadINDIConfigFile(true);
            this->getCCDParameters(true);
            ui->pbTakeImagePS->setEnabled(true);
            ui->gbCCDParamsMainCCD->setEnabled(true);
        }
    }
    this->loadPropertyList();

}

//-------------------------------------------------------------------
// slot that is called when new properties arrive
void MainWindow::loadPropertyList(void) {
    int i;
    QString* itemName;

    itemName = new QString();
    this->resetMainCamINDIPropertyGUIElements();
    ui->tabCamsP->setEnabled(true);
    if (this ->mainCamSelected == true) {
        ui->listMainCamProperties->clear();
        for (i = 0; i < camera_client->getNoOfProperties(true); i++) {
            itemName->clear();
            itemName->append((camera_client->getPropertyLabel(true,i)->toLatin1()));
            itemName->append(" (");
            itemName->append(camera_client->getPropertyGroup(true,i)->toLatin1());
            itemName->append(")");
            ui->listMainCamProperties->addItem(itemName->toLatin1());
            ui->lMainCCDPropMessage->setText("Property list changed!");
        }
    }
    if (this ->guiderCamSelected == true) {
        ui->listGuideCamProperties->clear();
        for (i = 0; i < camera_client->getNoOfProperties(false); i++) {
            itemName->clear();
            itemName->append((camera_client->getPropertyLabel(false,i)->toLatin1()));
            itemName->append(" (");
            itemName->append(camera_client->getPropertyGroup(false,i)->toLatin1());
            itemName->append(")");
            ui->listGuideCamProperties->addItem(itemName->toLatin1());
            ui->lGuideCCDPropMessage->setText("Property list changed!");
        }
    }
}

//------------------------------------------------------------------
// disconnect guidecam from INDI server
void MainWindow::disconnectFromINDIServer(void) {

    if ((this->ccdGuiderCameraIsAcquiring==false) && (this->ccdMainCameraIsAcquiring== false)) {
        this->camera_client->disconnectFromServer();
        ui->pbConnectToServer->setEnabled(true);
        ui->pbDisconnectFromServer->setEnabled(false);
        ui->pbKillINDIServer->setEnabled(true);
        ui->cbIndiIsUp->setChecked(false);
        ui->pbExpose->setEnabled(false);
        ui->pbStopExposure->setEnabled(false);
        ui->gbStartGuiderCCDINDI->setEnabled(true);
        ui->gbStartMainCCDINDI->setEnabled(true);
        ui->tabCamsP->setEnabled(false);
        ui->listMainCamProperties->clear();
        ui->listGuideCamProperties->clear();
        ui->leFrameSizeXMainCCD->clear();
        ui->leFrameSizeYMainCCD->clear();
        ui->leFrameSizeX->clear();
        ui->leFrameSizeY->clear();
        ui->lePixelSizeX->clear();
        ui->lePixelSizeY->clear();
        ui->lePixelSizeXMainCCD->clear();
        ui->lePixelSizeYMainCCD->clear();
        ui->lcdBitDepthMainCCD->display(0);
        ui->lcdBitDepth->display(0);
        this->storeMainCCDData();
        this->storeGuiderCCDData();
    }
}

//------------------------------------------------------------------
// a private little function that tries to find out about the PID of the
// INDI-server; the system call logs the process ID in a file .INDIpid.tsl
void MainWindow::findOutAboutINDIServerPID(void) {
    system("pidof indiserver > .INDIPID.tsl &");
    sleep(1);
}

//------------------------------------------------------------------
// this one kills a running INDI-server
void MainWindow::killRunningINDIServer(void) {
    QString *myCommand;

    std::ifstream infile(".INDIPID.tsl");
    std::string line;   // define a line that is read
    std::getline(infile, line);     // read that line
    if (line.length() > 0) {
        myCommand=new QString("kill -9 ");
        myCommand->append(line.data());
        myCommand->append(" &");
        system(myCommand->toLatin1());
        delete myCommand;
        system("rm -rf .INDIPID.tsl");
    }
    infile.close(); // close the file
    ui->pbKillINDIServer->setEnabled(false);
    ui->pbStartINDIServer->setEnabled(false);
    ui->gbStartGuiderCCDINDI->setEnabled(true);
    ui->gbStartMainCCDINDI->setEnabled(true);
    ui->pbConnectToServer->setEnabled(false);
    ui->gbCCDParams->setEnabled(false);
    ui->gbCCDParamsMainCCD->setEnabled(false);
    ui->rbMainCCD->setEnabled(false);
    ui->rbGuiderCCD->setEnabled(false);
    ui->leCam1Name->clear();
    ui->leCam2Name->clear();
    ui->rbMainCCD->setEnabled(false);
    ui->rbMainCCD->setEnabled(false);
    ui->rbMainCCD->setAutoExclusive(false);
    ui->rbGuiderCCD->setAutoExclusive(false);
    ui->rbMainCCD->setChecked(false);
    ui->rbGuiderCCD->setChecked(false);
    ui->rbMainCCD->setAutoExclusive(true);
    ui->rbGuiderCCD->setAutoExclusive(true);
    this->mainCamDriverName->clear();
    this->guideCamDriverName->clear();
    this->cam1Selected = false;
    this->cam2Selected = false;
    this->mainCamSelected = false;
    this->guiderCamSelected = false;
    this->setINDIrbuttons(true);
}

//------------------------------------------------------------------
// if a message from INDI is received by the camera class, process the signal and display it
void MainWindow::handleServerMessage(void) {
    QString *indiMesg;

    indiMesg=new QString(camera_client->getINDIServerMessage()->toLatin1());
    if (indiMesg->isEmpty()==false) {
        ui->textEditINDIMsgs->appendPlainText(indiMesg->toLatin1());
    }
    delete indiMesg;
}
//------------------------------------------------------------------
// deploy a system command to start an INDI server locally with standard parameters.
// type of server is defined by radiobuttons ...
void MainWindow::deployINDICommand(void) {
    int retval = 0;
    QString *indiServerCommand;

    if ((this->cam1Selected == false) && (this->cam2Selected==false)) {
        qDebug() << "No Cameras selected ...";
        return;
    }
    indiServerCommand = new QString("indiserver -v ");
    if (this->mainCamDriverName->compare(this->guideCamDriverName) == 0) {
        if (this->mainCamDriverName->isEmpty() == false) {
            indiServerCommand->append(this->mainCamDriverName);
            indiServerCommand->append(" &");
        }
    } else {
        indiServerCommand->append(this->mainCamDriverName);
        indiServerCommand->append(" ");
        indiServerCommand->append(this->guideCamDriverName);
        indiServerCommand->append(" &");
    }
    retval = system(indiServerCommand->toLatin1());
    if (retval == 0) {
        ui->pbStartINDIServer->setEnabled(false);
        ui->pbKillINDIServer->setEnabled(true);
        ui->pbConnectToServer->setEnabled(true);
        this->setINDIrbuttons(false);
        QCoreApplication::processEvents(QEventLoop::AllEvents,2000);   // process events before sleeping for a second
    }
    this->waitForNMSecs(1000);
    qDebug() << "Command to start INDIServer: " << indiServerCommand->toLatin1();
    delete indiServerCommand;
    this->findOutAboutINDIServerPID(); // store the PID in a file
}

//------------------------------------------------------------------
// a slot for selecting a guider camera
void MainWindow::selectGuiderCamDriverName(void) {
    if (ui->rbGuiderCamNone->isChecked() == false) {
        if (ui->rbApogee->isChecked()== true) {
            this->guideCamDriverName->append("indi_apogee_ccd");
        }
        if (ui->rbATIK->isChecked()== true) {
            this->guideCamDriverName->append("indi_atik_ccd");
        }
        if (ui->rbFLI->isChecked()== true) {
            this->guideCamDriverName->append("indi_fli_ccd");
        }
        if (ui->rbINova->isChecked()== true) {
            this->guideCamDriverName->append("indi_nova_ccd");
        }
        if (ui->rbMeadeDSI->isChecked()== true) {
            this->guideCamDriverName->append("indi_dsi_ccd");
        }
        if (ui->rbQSI->isChecked()== true) {
            this->guideCamDriverName->append("indi_qsi_ccd");
        }
        if (ui->rbSBIG->isChecked()== true) {
            this->guideCamDriverName->append("indi_sbig_ccd");
        }
        if (ui->rbQHYINDI->isChecked()== true) {
            this->guideCamDriverName->append("indi_qhy_ccd");
        }
        if (ui->rbZWOINDI->isChecked()== true) {
            this->guideCamDriverName->append("indi_asi_ccd");
        }
        if (ui->rbV4L2INDI->isChecked()== true) {
            this->guideCamDriverName->append("indi_v4l2_ccd");
        }
        if (ui->rbMoravian->isChecked()== true) {
            this->guideCamDriverName->append("indi_mi_ccd");
        }
        if (ui->rbSLXPress->isChecked() == true) {
            this->guideCamDriverName->append("indi_sx_ccd");
        }
        if (ui->rbToUpTek->isChecked() == true) {
            this->guideCamDriverName->append("indi_toupcam_ccd");
        }
        if (ui->rbNightscape->isChecked() == true) {
            this->guideCamDriverName->append("indi_nightscape_ccd");
        }
        if (ui->rbAltair->isChecked() == true) {
            this->guideCamDriverName->append("indi_altair_ccd");
        }
        this->cam1Selected = true;
        ui->gbStartGuiderCCDINDI->setEnabled(false);
    } else {
        this->cam1Selected = false;
        this->guideCamDriverName->clear();
        ui->gbStartGuiderCCDINDI->setEnabled(false);
    }
    if ((ui->gbStartGuiderCCDINDI->isEnabled() == false) && (ui->gbStartMainCCDINDI->isEnabled() == false)) {
        ui->pbStartINDIServer->setEnabled(true); // in this case, both selection buttons were already pressed ...
    }
}

//------------------------------------------------------------------
void MainWindow::selectMainCamDriverName(void) {
    if (ui->rbMainCCDNone->isChecked() == false) {
        if (ui->rbZwoMainCCD->isChecked()== true) {
            this->mainCamDriverName->append("indi_asi_ccd");
        }
        if (ui->rbDSLR->isChecked()==true) {
            this->mainCamDriverName->append("indi_gphoto_ccd");
        }
        if (ui->rbATIKMainCCD->isChecked() == true) {
            this->mainCamDriverName->append("indi_atik_ccd");
        }
        if (ui->rbMoravianMainCCD->isChecked() == true) {
            this->mainCamDriverName->append("indi_mi_ccd");
        }
        if (ui->rbQHYMainCCD->isChecked() == true) {
            this->mainCamDriverName->append("indi_qhy_ccd");
        }
        this->cam2Selected = true;
        ui->gbStartMainCCDINDI->setEnabled(false);
    } else {
        this->cam2Selected = false;
        this->mainCamDriverName->clear();
        ui->gbStartMainCCDINDI->setEnabled(false);
    }
    if ((ui->gbStartGuiderCCDINDI->isEnabled() == false) && (ui->gbStartMainCCDINDI->isEnabled() == false)) {
        ui->pbStartINDIServer->setEnabled(true); // in this case, both selection buttons were already pressed ...
    }
}

//------------------------------------------------------------------
// slot that erases the log window for INDI
void MainWindow::clearINDILog(void) {
    ui->textEditINDIMsgs->clear();
}

//------------------------------------------------------------------
// slots to set INDI properties from the main list widget on properties
void MainWindow::mainCamPropertySelected(void) {
    int idx, i;
    INDI_PROPERTY_TYPE ipt;

    qDebug() << "Selecting property ...";
    QCoreApplication::processEvents(QEventLoop::AllEvents,1000);
    this->resetMainCamINDIPropertyGUIElements();
    idx = ui->listMainCamProperties->currentRow();
    ipt = camera_client->getPropertyType(true,idx);
    switch(ipt) {
    case INDI_NUMBER:
        if (camera_client->getRWPermission(true,idx,true) == true) { // ask for permission to read value
            ui->lwMainCCDNumberNames->setEnabled(true);
            for (i=0; i < camera_client->getNoOfValuesInProperty(true,idx); i++) {
                ui->lwMainCCDNumberNames->addItem(camera_client->getNumberPropertyItemLabel(true, idx, i)->toLatin1());
            }
        } else {
            qDebug() << "cannot read value ...";
        }
        break;
    case INDI_TEXT:
        if (camera_client->getRWPermission(true,idx,true) == true) { // ask for permission to read value
            ui->lwMainCCDTextNames->setEnabled(true);
            for (i=0; i < camera_client->getNoOfValuesInProperty(true,idx); i++) {
                ui->lwMainCCDTextNames->addItem(camera_client->getTextPropertyItemLabel(true, idx, i)->toLatin1());
            }
        } else {
            qDebug() << "cannot read value ...";
        }
        break;
    case INDI_SWITCH:
        if ((camera_client->getRWPermission(true,idx,true) == true)) { // as for permission to read value; discconnecting is not possible
            ui->lwMainCCDSwitchNames->setEnabled(true);
            this->getINDISwitchRules(true);
            for (i=0; i < camera_client->getNoOfValuesInProperty(true,idx); i++) {
                ui->lwMainCCDSwitchNames->addItem(camera_client->getSwitchPropertyItemLabel(true, idx, i)->toLatin1());
            }
        } else {
            qDebug() << "cannot read value ...";
        }
        break;
    case INDI_LIGHT:
        if (camera_client->getRWPermission(true,idx,true) == true) { // as for permission to read value
            ui->lwMainCCDLightNames->setEnabled(true);
            for (i=0; i < camera_client->getNoOfValuesInProperty(true,idx); i++) {
                ui->lwMainCCDLightNames->addItem(camera_client->getLightPropertyItemLabel(true, idx,i)->toLatin1());
            }
        } else {
            qDebug() << "cannot read value ...";
        }
        break;
    case INDI_BLOB:
        if (camera_client->getRWPermission(true,idx,true) == true) { // as for permission to read value
            ui->cbMainCCDIsBLOB->setChecked(true);
        } else {
            qDebug() << "cannot read value ...";
        }
        break;
    case INDI_UNKNOWN:
        qDebug() << "Selected an unknown property...";
        break;
    }
}

//------------------------------------------------------------------
// slots to set INDI properties from the main list widget on properties
void MainWindow::guideCamPropertySelected(void) {
    int idx, i;
    INDI_PROPERTY_TYPE ipt;

    QCoreApplication::processEvents(QEventLoop::AllEvents,1000);
    this->resetGuideCamINDIPropertyGUIElements();
    idx = ui->listGuideCamProperties->currentRow();
    ipt = camera_client->getPropertyType(false,idx);
    switch(ipt) {
    case INDI_NUMBER:
        if (camera_client->getRWPermission(false,idx,true) == true) { // ask for permission to read value
            ui->lwGuideCCDNumberNames->setEnabled(true);
            for (i=0; i < camera_client->getNoOfValuesInProperty(false,idx); i++) {
                ui->lwGuideCCDNumberNames->addItem(camera_client->getNumberPropertyItemLabel(false, idx, i)->toLatin1());
            }
        } else {
            qDebug() << "cannot read value ...";
        }
        break;
    case INDI_TEXT:
        if (camera_client->getRWPermission(false,idx,true) == true) { // ask for permission to read value
            ui->lwGuideCCDTextNames->setEnabled(true);
            for (i=0; i < camera_client->getNoOfValuesInProperty(false,idx); i++) {
                ui->lwGuideCCDTextNames->addItem(camera_client->getTextPropertyItemLabel(false, idx, i)->toLatin1());
            }
        } else {
            qDebug() << "cannot read value ...";
        }
        break;
    case INDI_SWITCH:
        if ((camera_client->getRWPermission(false,idx,true) == true)) { // as for permission to read value; discconnecting is not possible
            ui->lwGuideCCDSwitchNames->setEnabled(true);
            this->getINDISwitchRules(false);
            for (i=0; i < camera_client->getNoOfValuesInProperty(false,idx); i++) {
                ui->lwGuideCCDSwitchNames->addItem(camera_client->getSwitchPropertyItemLabel(false, idx, i)->toLatin1());
            }
        } else {
            qDebug() << "cannot read value ...";
        }
        break;
    case INDI_LIGHT:
        if (camera_client->getRWPermission(false,idx,true) == true) { // as for permission to read value
            ui->lwGuideCCDLightNames->setEnabled(true);
            for (i=0; i < camera_client->getNoOfValuesInProperty(false,idx); i++) {
                ui->lwMainCCDLightNames->addItem(camera_client->getLightPropertyItemLabel(false, idx,i)->toLatin1());
            }
        } else {
            qDebug() << "cannot read value ...";
        }
        break;
    case INDI_BLOB:
        if (camera_client->getRWPermission(false,idx,true) == true) { // as for permission to read value
            ui->cbGuideCCDIsBLOB->setChecked(true);
        } else {
            qDebug() << "cannot read value ...";
        }
        break;
    case INDI_UNKNOWN:
        qDebug() << "Selected an unknown property...";
        break;
    }
}

//------------------------------------------------------------------
// slots to select single INDI Property values for changing and display
void MainWindow::mainCamSetINDINumberProperties(void) {
    int idx, cnt;
    double dispval, hmax, hmin, hstep;
    INumber np;

    ui->sbMainCCDINDINumberSet->setEnabled(false);
    ui->pbMainCCDNumberValueSet->setEnabled(false);
    idx = ui->listMainCamProperties->currentRow();
    cnt = ui->lwMainCCDNumberNames->currentRow();
    np = camera_client->getCurrentNumber(true, idx, cnt);
    dispval = np.value;
    ui->lcdMainCCDNumberValue->display(dispval);
    if (camera_client->getRWPermission(true,idx,false) == true) { // as for permission to set value
        ui->sbMainCCDINDINumberSet->setEnabled(true);
        ui->pbMainCCDNumberValueSet->setEnabled(true);
        hmax = np.max;
        hmin = np.min;
        if (np.step > 0) {
            hstep = np.step;
        } else {
            hstep = 0.01;
        }
        ui->sbMainCCDINDINumberSet->setMaximum(hmax);
        ui->sbMainCCDINDINumberSet->setMinimum(hmin);
        ui->sbMainCCDINDINumberSet->setSingleStep(hstep);
        ui->sbMainCCDINDINumberSet->setValue(dispval);
    }
}

//------------------------------------------------------------------
// slots to select single INDI Property values for changing and display
void MainWindow::guideCamSetINDINumberProperties(void) {
    int idx, cnt;
    double dispval, hmax, hmin, hstep;
    INumber np;

    ui->sbGuideCCDINDINumberSet->setEnabled(false);
    ui->pbGuideCCDNumberValueSet->setEnabled(false);
    idx = ui->listGuideCamProperties->currentRow();
    cnt = ui->lwGuideCCDNumberNames->currentRow();
    np = camera_client->getCurrentNumber(false, idx, cnt);
    dispval = np.value;
    ui->lcdGuideCCDNumberValue->display(dispval);
    if (camera_client->getRWPermission(false,idx,false) == true) { // as for permission to set value
        ui->sbGuideCCDINDINumberSet->setEnabled(true);
        ui->pbGuideCCDNumberValueSet->setEnabled(true);
        hmax = np.max;
        hmin = np.min;
        if (np.step > 0) {
            hstep = np.step;
        } else {
            hstep = 0.01;
        }
        ui->sbGuideCCDINDINumberSet->setMaximum(hmax);
        ui->sbGuideCCDINDINumberSet->setMinimum(hmin);
        ui->sbGuideCCDINDINumberSet->setSingleStep(hstep);
        ui->sbGuideCCDINDINumberSet->setValue(dispval);
    }
}

//------------------------------------------------------------------
// slot that actively sets a number to the INDI server propertyvector
void MainWindow::mainCamSendINDINumber(void) {
    int idx, cnt;
    double val;
    INumber np;

    idx = ui->listMainCamProperties->currentRow();
    cnt = ui->lwMainCCDNumberNames->currentRow();
    val = ui->sbMainCCDINDINumberSet->value();
    if (camera_client->getRWPermission(true,idx,false) == true) { // as for permission to set value
        camera_client->setCurrentNumberValue(true, idx, cnt, val);
    }
    np = camera_client->getCurrentNumber(true, idx, cnt);
    val = np.value;
    ui->lcdMainCCDNumberValue->display(val);
    ui->pbMainCCDNumberValueSend->setEnabled(true);
}

//------------------------------------------------------------------
// slot that sends the current number numberproperty vector to the server
void MainWindow::mainCamSetINDINumberOnServer(void) {
    camera_client->sendCurrentNumberValueCompound();
    this->storeMainCCDData(); // if resolution or depth are changed, this has to be made known
    this->getCCDParameters(true);
}

//------------------------------------------------------------------
// slot that actively sends a number to the INDI server
void MainWindow::guideCamSendINDINumber(void) {
    int idx, cnt;
    double val;
    INumber np;

    idx = ui->listGuideCamProperties->currentRow();
    cnt = ui->lwGuideCCDNumberNames->currentRow();
    val = ui->sbGuideCCDINDINumberSet->value();
    if (camera_client->getRWPermission(false,idx,false) == true) { // as for permission to set value
        camera_client->setCurrentNumberValue(false, idx, cnt, val);
    }
    np = camera_client->getCurrentNumber(false, idx, cnt);
    val = np.value;
    ui->lcdGuideCCDNumberValue->display(val);
    ui->pbGuideCCDNumberValueSend->setEnabled(true);
}

//------------------------------------------------------------------
// slot that sends the current number numberproperty vector to the server
void MainWindow::guideCamSetINDINumberOnServer(void) {
    camera_client->sendCurrentNumberValueCompound();
    this->storeGuiderCCDData(); // if resolution or depth are changed, this has to be made known
    this->getCCDParameters(false);
}

//------------------------------------------------------------------
void MainWindow::mainCamSetINDITextProperties(void) {
    int idx, cnt;
    QString *dispText;

    ui->leMainCCDTextValue->setEnabled(false);
    ui->leMainCCDTextValueSet->setEnabled(false);
    idx = ui->listMainCamProperties->currentRow();
    cnt = ui->lwMainCCDTextNames->currentRow();
    if (camera_client->getRWPermission(true,idx,true) == true) { // as for permission to read value
        dispText = new QString(camera_client->getCurrentText(true, idx, cnt)->toLatin1());
        ui->leMainCCDTextValue->setText(dispText->toLatin1());
        delete dispText;
        if (camera_client->getRWPermission(true,idx,false) == true) { // as for permission to set value
            ui->leMainCCDTextValueSet->setEnabled(true);
            ui->pbMainCCDTextValueSet->setEnabled(true);
        }
    }
}

//------------------------------------------------------------------
void MainWindow::guideCamSetINDITextProperties(void) {
    int idx, cnt;
    QString *dispText;

    ui->leGuideCCDTextValue->setEnabled(false);
    ui->leGuideCCDTextValueSet->setEnabled(false);
    idx = ui->listGuideCamProperties->currentRow();
    cnt = ui->lwGuideCCDTextNames->currentRow();
    if (camera_client->getRWPermission(false,idx,true) == true) { // as for permission to read value
        dispText = new QString(camera_client->getCurrentText(false, idx, cnt)->toLatin1());
        ui->leGuideCCDTextValue->setText(dispText->toLatin1());
        delete dispText;
        if (camera_client->getRWPermission(false,idx,false) == true) { // as for permission to set value
            ui->leGuideCCDTextValueSet->setEnabled(true);
            ui->pbGuideCCDTextValueSet->setEnabled(true);
        }
    }
}

//------------------------------------------------------------------
// slot that actively sends a text to the INDI server
void MainWindow::mainCamSendINDIText(void) {
    int idx, cnt;
    QString* txt;

    idx = ui->listMainCamProperties->currentRow();
    cnt = ui->lwMainCCDTextNames->currentRow();
    if (camera_client->getRWPermission(true,idx,false) == true) { // as for permission to set value
        txt = new QString(ui->leMainCCDTextValueSet->text().toLatin1());
        camera_client->setCurrentTextValue(true, idx, cnt, *txt);
        delete txt;
        this->mainCamSetINDITextProperties();
    }
    ui->leMainCCDTextValue->clear();
}

//------------------------------------------------------------------
// slot that actively sends a text to the INDI server
void MainWindow::guideCamSendINDIText(void) {
    int idx, cnt;
    QString* txt;

    idx = ui->listGuideCamProperties->currentRow();
    cnt = ui->lwGuideCCDTextNames->currentRow();
    if (camera_client->getRWPermission(false,idx,false) == true) { // as for permission to set value
        txt = new QString(ui->leGuideCCDTextValueSet->text().toLatin1());
        camera_client->setCurrentTextValue(false, idx, cnt, *txt);
        delete txt;
        this->guideCamSetINDITextProperties();
    }
    ui->leGuideCCDTextValue->clear();
}
//------------------------------------------------------------------
void MainWindow::getINDISwitchRules(bool isMainCCD) {
    int idx = 0;
    ISRule possibilities;

    if (isMainCCD == true) {
        idx = ui->listMainCamProperties->currentRow();
    } else {
        idx = ui->listGuideCamProperties->currentRow();
    }
    if (camera_client->getRWPermission(isMainCCD,idx,true) == true) { // as for permission to read value
       possibilities = camera_client->getSwitchRules(isMainCCD,idx);
       switch (possibilities) {
       case ISR_1OFMANY:
            if (isMainCCD == true) {
                ui->lMainCCDINDISwitchType->setText("One switch has to be ON!");
            } else {
                ui->lGuideCCDINDISwitchType->setText("One switch has to be ON!");
            }
            break;
       case ISR_ATMOST1:
            if (isMainCCD == true) {
                ui->lMainCCDINDISwitchType->setText("One or no switch can be ON!");
            } else {
                ui->lGuideCCDINDISwitchType->setText("One or no switch can be ON!");
            }
            break;
       case ISR_NOFMANY:
            if (isMainCCD == true) {
                ui->lMainCCDINDISwitchType->setText("Many switches can be ON!");
            } else {
                ui->lGuideCCDINDISwitchType->setText("Many switches can be ON!");
            }
           break;
        }
    }
}

//------------------------------------------------------------------
void MainWindow::mainCamSetINDISwitchProperties(void) {
    int idx, cnt;
    ISState status;
    bool isOn;

    ui->cbMainCCDINDISwitchOn->setEnabled(false);
    ui->cbMainCCDSwitchReceived->setChecked(false);
    idx = ui->listMainCamProperties->currentRow();
    cnt = ui->lwMainCCDSwitchNames->currentRow();
    if (camera_client->getRWPermission(true,idx,true) == true) { // as for permission to read value
        status = camera_client->getCurrentSwitch(true, idx, cnt);
        if (status == ISS_ON) {
            isOn=true;
        } else {
            isOn=false;
        }
        ui->cbMainCCDINDISwitchOn->setEnabled(true);
        ui->pbMainCCDSetINDISwitch->setEnabled(true);
        ui->cbMainCCDINDISwitchOn->setChecked(isOn);
        if (camera_client->getRWPermission(true,idx,false) == true) { // as for permission to write value
        }
    }
}

//------------------------------------------------------------------
void MainWindow::guideCamSetINDISwitchProperties(void) {
    int idx, cnt;
    ISState status;
    bool isOn;

    ui->cbGuideCCDINDISwitchOn->setEnabled(false);
    ui->cbGuideCCDSwitchReceived->setChecked(false);
    idx = ui->listGuideCamProperties->currentRow();
    cnt = ui->lwGuideCCDSwitchNames->currentRow();
    if (camera_client->getRWPermission(false,idx,true) == true) { // as for permission to read value
        status = camera_client->getCurrentSwitch(false, idx, cnt);
        if (status == ISS_ON) {
            isOn=true;
        } else {
            isOn=false;
        }
        ui->cbGuideCCDINDISwitchOn->setEnabled(true);
        ui->pbGuideCCDSetINDISwitch->setEnabled(true);
        ui->cbGuideCCDINDISwitchOn->setChecked(isOn);
        if (camera_client->getRWPermission(false,idx,false) == true) { // as for permission to write value
        }
    }
}
//------------------------------------------------------------------
// slot that actively sends a switch to the INDI server
void MainWindow::mainCamSendINDISwitch(void) {
    int idx, cnt;
    bool status;

    idx = ui->listMainCamProperties->currentRow();
    cnt = ui->lwMainCCDSwitchNames->currentRow();
    if (camera_client->getRWPermission(true,idx,false) == true) { // as for permission to set value
        if (ui->cbMainCCDINDISwitchOn->isChecked() == true) {
            status = true;
        } else {
            status = false;
        }
        camera_client->setCurrentSwitch(true, idx, cnt, status);
    } else {
        qDebug() << "cannot set switch value...";
    }
}

//------------------------------------------------------------------
// slot that actively sends a switch to the INDI server
void MainWindow::guideCamSendINDISwitch(void) {
    int idx, cnt;
    bool status;

    idx = ui->listGuideCamProperties->currentRow();
    cnt = ui->lwGuideCCDSwitchNames->currentRow();
    if (camera_client->getRWPermission(false,idx,false) == true) { // as for permission to set value
        if (ui->cbGuideCCDINDISwitchOn->isChecked() == true) {
            status = true;
        } else {
            status = false;
        }
        camera_client->setCurrentSwitch(false, idx, cnt, status);
    } else {
        qDebug() << "cannot set switch value...";
    }
}

//------------------------------------------------------------------
void MainWindow::resetMainCamINDIPropertyGUIElements(void) {
    ui->lwMainCCDTextNames->clear();
    ui->lwMainCCDTextNames->setEnabled(false);
    ui->leMainCCDTextValue->clear();
    ui->leMainCCDTextValueSet->clear();
    ui->leMainCCDTextValueSet->setEnabled(false);
    ui->pbMainCCDTextValueSet->setEnabled(false);
    ui->cbMainCCDTextReceived->setChecked(false);
    ui->cbMainCCDTextReceived->setEnabled(false);
    ui->lMainCCDINDITextOK->setEnabled(false);

    ui->lwMainCCDNumberNames->clear();
    ui->lwMainCCDNumberNames->setEnabled(false);
    ui->lcdMainCCDNumberValue->display(0);
    ui->sbMainCCDINDINumberSet->setValue(0);
    ui->sbMainCCDINDINumberSet->setEnabled(false);
    ui->pbMainCCDNumberValueSet->setEnabled(false);
    ui->pbMainCCDNumberValueSend->setEnabled(false);
    ui->cbMainCCDNumberReceived->setChecked(false);
    ui->cbMainCCDNumberReceived->setEnabled(false);
    ui->lMainCCDINDINumberOK->setEnabled(false);

    ui->lwMainCCDSwitchNames->clear();
    ui->lwMainCCDSwitchNames->setEnabled(false);
    ui->lMainCCDINDISwitchType->clear();
    ui->cbMainCCDINDISwitchOn->setChecked(false);
    ui->cbMainCCDINDISwitchOn->setEnabled(false);
    ui->pbMainCCDSetINDISwitch->setEnabled(false);
    ui->cbMainCCDSwitchReceived->setChecked(false);
    ui->cbMainCCDSwitchReceived->setEnabled(false);
    ui->lMainCCDINDISwitchOK->setEnabled(false);

    ui->lwMainCCDLightNames->clear();
    ui->lwMainCCDLightNames->setEnabled(false);
    ui->cbMainCCDLightOn->setChecked(false);

    ui->cbMainCCDIsBLOB->setChecked(false);

    ui->lMainCCDPropMessage->clear();
}

//------------------------------------------------------------------
void MainWindow::resetGuideCamINDIPropertyGUIElements(void) {
    ui->lwGuideCCDTextNames->clear();
    ui->lwGuideCCDTextNames->setEnabled(false);
    ui->leGuideCCDTextValue->clear();
    ui->leGuideCCDTextValueSet->clear();
    ui->leGuideCCDTextValueSet->setEnabled(false);
    ui->pbGuideCCDTextValueSet->setEnabled(false);
    ui->cbGuideCCDTextReceived->setChecked(false);
    ui->cbGuideCCDTextReceived->setEnabled(false);
    ui->lGuideCCDINDITextOK->setEnabled(false);

    ui->lwGuideCCDNumberNames->clear();
    ui->lwGuideCCDNumberNames->setEnabled(false);
    ui->lcdGuideCCDNumberValue->display(0);
    ui->sbGuideCCDINDINumberSet->setValue(0);
    ui->sbGuideCCDINDINumberSet->setEnabled(false);
    ui->pbGuideCCDNumberValueSet->setEnabled(false);
    ui->pbGuideCCDNumberValueSend->setEnabled(false);
    ui->cbGuideCCDNumberReceived->setChecked(false);
    ui->cbGuideCCDNumberReceived->setEnabled(false);
    ui->lGuideCCDINDINumberOK->setEnabled(false);

    ui->lwGuideCCDSwitchNames->clear();
    ui->lwGuideCCDSwitchNames->setEnabled(false);
    ui->lGuideCCDINDISwitchType->clear();
    ui->cbGuideCCDINDISwitchOn->setChecked(false);
    ui->cbGuideCCDINDISwitchOn->setEnabled(false);
    ui->pbGuideCCDSetINDISwitch->setEnabled(false);
    ui->cbGuideCCDSwitchReceived->setChecked(false);
    ui->cbGuideCCDSwitchReceived->setEnabled(false);
    ui->lGuideCCDINDISwitchOK->setEnabled(false);

    ui->lwGuideCCDLightNames->clear();
    ui->lwGuideCCDLightNames->setEnabled(false);
    ui->cbGuideCCDLightOn->setChecked(false);

    ui->cbGuideCCDIsBLOB->setChecked(false);

    ui->lGuideCCDPropMessage->clear();
}
//------------------------------------------------------------------
void MainWindow::indicateNumberOnINDIServerMainCCD(void) {
    ui->cbMainCCDNumberReceived->setChecked(true);
    ui->lMainCCDINDINumberOK->setEnabled(true);
}

//------------------------------------------------------------------
void MainWindow::indicateTextOnINDIServerMainCCD(void) {
    ui->cbMainCCDTextReceived->setChecked(true);
    ui->lMainCCDINDITextOK->setEnabled(true);
}

//------------------------------------------------------------------
void MainWindow::indicateSwitchOnINDIServerMainCCD(void) {
    ui->cbMainCCDSwitchReceived->setChecked(true);
    ui->lMainCCDINDISwitchOK->setEnabled(true);
}

//------------------------------------------------------------------
void MainWindow::indicateNumberOnINDIServerGuiderCCD(void) {
    ui->cbGuideCCDNumberReceived->setChecked(true);
    ui->lGuideCCDINDINumberOK->setEnabled(true);
}

//------------------------------------------------------------------
void MainWindow::indicateTextOnINDIServerGuiderCCD(void) {
    ui->cbGuideCCDTextReceived->setChecked(true);
    ui->lGuideCCDINDITextOK->setEnabled(true);
}

//------------------------------------------------------------------
void MainWindow::indicateSwitchOnINDIServerGuiderCCD(void) {
    ui->cbGuideCCDSwitchReceived->setChecked(true);
    ui->lGuideCCDINDISwitchOK->setEnabled(true);
}

//------------------------------------------------------------------
void MainWindow::deployINDIMsgDlg(void) {
    QMessageBox noSaveBoxMsg;

    noSaveBoxMsg.setWindowTitle("INDI Server message");
    noSaveBoxMsg.setText("Server cannot store configuration - functionality not available.");
    noSaveBoxMsg.exec();
}

//------------------------------------------------------------------
void MainWindow::saveMainCCDConfig(void) {
    camera_client->saveINDIConfigFile(true);
    this->storeMainCCDData(); // if resolution or depth are changed, this has to be made known
    this->getCCDParameters(true);
}

//------------------------------------------------------------------
void MainWindow::saveGuideCCDConfig(void) {
    camera_client->saveINDIConfigFile(false);
    this->storeGuiderCCDData(); // if resolution or depth are changed, this has to be made known
    this->getCCDParameters(false);
}
//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
// routines for handling the ccd - camera
//------------------------------------------------------------------
//------------------------------------------------------------------
// send an exposure time to the camera and tell INDI to take an image
// an event is triggered once these data were received
void MainWindow::takeSingleGuiderCamShot(void) {
   int exptime;
   exptime = ui->sbExposureTime->value();
   camera_client->takeExposure(exptime,false);
   this->waitForNMSecs(250);
}

//------------------------------------------------------------------
// send an exposure time to the camera and tell INDI to tak    void changeCCDGain(void);e an image
// an event is triggered once these data were received
void MainWindow::takeSingleMainCamShot(void) {
   int exptime;
   exptime = ui->sbExposureTime->value();
   camera_client->takeExposure(exptime,true);
   this->waitForNMSecs(250);
}
//------------------------------------------------------------------
// prepare GUI for taking images; once an image was received, a
// new one is requested in "displayGuideCamImage"
void MainWindow::startCCDAcquisition(void) {
    this->ccdGuiderCameraIsAcquiring=true;
    ui->pbExpose->setEnabled(false);
    ui->pbStopExposure->setEnabled(true);
    this->takeSingleGuiderCamShot();
    this->waitForNMSecs(200);
    ui->pbSelectGuideStar->setEnabled(true);
    ui->pbDisconnectFromServer->setEnabled(false);
    ui->tabImageProc->setEnabled(true);
}

//------------------------------------------------------------------
// set a flag so that no new image is requested in "displayGuideCamImage"
void MainWindow::stopCCDAcquisition(void) {

    this->ccdGuiderCameraIsAcquiring=false;
    ui->pbStopExposure->setEnabled(false);
    ui->pbDisconnectFromServer->setEnabled(true);
    this->camImageWasReceived=false;
    ui->tabImageProc->setEnabled(false);
    ui->tabCCDCal->setEnabled(false);
    ui->tabGuide->setEnabled(false);
    ui->cbSaturation->setChecked(false);
}

//------------------------------------------------------------------
// same as "stopCCDAcquisition", but it also waits for the
// image to be received. this is indicated when "camImageWasReceived"
// is set to "true" as when a signal "imageAvailable" is emitted by
// the camera-class. this one is connected to "displayGuideCamImage",
// where images are further handled and "camImageWasReceived" is also
// set to "true". return "true" if a final image was acquired and
// "false" if a timeout ocurred ...
bool MainWindow::abortCCDAcquisition(void) {
    QElapsedTimer *timeElapsedLocal;
    long maxTime;

    this->ccdGuiderCameraIsAcquiring=false;
    ui->pbStopExposure->setEnabled(false);
    ui->pbDisconnectFromServer->setEnabled(true);
    this->camImageWasReceived=false;
    timeElapsedLocal = new QElapsedTimer();
    timeElapsedLocal->start();
    maxTime = ui->sbExposureTime->value()*5000; // wait for a maximum of 5* the exposure time for a last image
    while (this->camImageWasReceived==false) {
        QCoreApplication::processEvents(QEventLoop::AllEvents,maxTime);   // process events while waiting for the last image
        if (timeElapsedLocal->elapsed() > maxTime) {
            delete timeElapsedLocal;
            return false;
        }
    }
    delete timeElapsedLocal;
    return true;
}

//------------------------------------------------------------------
// this one tells the camera class to store the ccd images
void MainWindow::enableCamImageStorage(void) {
    if (ui->cbStoreGuideCamImgs->isChecked()==true) {
        camera_client->setStoreImageFlag(true);
    } else {
        camera_client->setStoreImageFlag(false);
    }
}

//------------------------------------------------------------------
// does a lot - stores the camera image in the mainwindow class,
// displays the bigger image, but if a guidestar is selected, it also
// takes care of processing the preview image. also polls new images
// if the appropriate flag is set
void MainWindow::displayGuideCamImage(QPixmap *camPixmap) {
    int thrshld,beta;
    float newX, newY,alpha;
    bool medianOn, lpOn;

    this->camImageWasReceived= true;
    thrshld = ui->hsThreshold->value();
    alpha = ui->hsIContrast->value()/100.0;
    beta = ui->hsIBrightness->value();
    lpOn = ui->cbLowPass->isChecked();
    medianOn=ui->cbMedianFilter->isChecked(); // get parameters for guidestar-processing from GUI
       // now cope with different states and images needed ...

    if (g_AllData->getINDIState(false) == true) { // ... if the camera class is connected to the INDI server ...
        delete camImg;
        this->camImg = new QPixmap(*camPixmap);
        this->camView->addBgImage(*camImg); // receive the pixmap from the camera ...
        if (this->guidingState.calibrationIsRunning == true) { // autoguider is calibrating
            this->guidingState.calibrationImageReceived=true; // in calibration, this camera image is to be used
        } // we only take a single shot here
        if ((this->ccdGuiderCameraIsAcquiring==true) && (this->guidingState.guidingIsOn==false)) { // if the flag for taking another one is true ...
            this->waitForNMSecs(100);
            this->takeSingleGuiderCamShot(); // ... request another one from INDI
            this->waitForNMSecs(500);
        } else {
            ui->pbExpose->setEnabled(true); // if acquisition is disabled, set the GUI so that it can be enabled
        }
        if ((this->guidingState.guidingIsOn==true) && (this->guidingState.systemIsCalibrated==true)) { // if autoguiding is active and system is calibrated
            this->guidingState.noOfGuidingSteps++; // every odd one, corrections are applied ...
            ui->lcdGuidesteps->display((int)(this->guidingState.noOfGuidingSteps));
            this->guiding->doGuideStarImgProcessing(thrshld,medianOn, lpOn,alpha,beta,this->guidingFOVFactor,this->guidingState.guideStarSelected, true); // ... process the guide star subimage
            newX = g_AllData->getInitialStarPosition(2);
            newY = g_AllData->getInitialStarPosition(3); // the star centroid found in "doGuideStarImgProcessing" was stored in the global struct ...
            this->waitForNMSecs(2500);
            correctGuideStarPosition(newX,newY); // ... and is used to correct the position
        }
    }
}

//------------------------------------------------------------------
// retrieve parameters for the guide CCD from the camera class
 bool MainWindow::getCCDParameters(bool isMainCCD) {
    bool retrievalSuccess;
    QString letxt;
    float psx,psy;
    int fsx,fsy, bd;

    if (isMainCCD == false) {
        retrievalSuccess = camera_client->getCCDParameters(isMainCCD);
        if (retrievalSuccess==true) {
            qDebug() << "Getting data for guider...";
            psx=g_AllData->getCameraPixelSize(0,isMainCCD);
            psy=g_AllData->getCameraPixelSize(1,isMainCCD);
            fsx=(int)g_AllData->getCameraChipPixels(0,isMainCCD);
            fsy=(int)g_AllData->getCameraChipPixels(1,isMainCCD);
            qDebug() << fsx << "/" << fsy;
            bd = g_AllData->getCameraBitDepth(isMainCCD);
            letxt=QString::number((double)psx,'g',2);
            ui->lePixelSizeX->setText(letxt);
            letxt=QString::number((double)psy,'g',2);
            ui->lePixelSizeY->setText(letxt);
            letxt=QString::number(fsx);
            ui->leFrameSizeX->setText(letxt);
            letxt=QString::number(fsy);
            ui->leFrameSizeY->setText(letxt);
            ui->lcdBitDepth->display(bd);
        }
    } else {
        retrievalSuccess = camera_client->getCCDParameters(isMainCCD);
        if (retrievalSuccess==true) {
            qDebug() << "Getting data for main ccd...";
            psx=g_AllData->getCameraPixelSize(0,isMainCCD);
            psy=g_AllData->getCameraPixelSize(1,isMainCCD);
            fsx=(int)g_AllData->getCameraChipPixels(0,isMainCCD);
            fsy=(int)g_AllData->getCameraChipPixels(1,isMainCCD);
            qDebug() << fsx << "/" << fsy;
            bd = g_AllData->getCameraBitDepth(isMainCCD);
            letxt=QString::number((double)psx,'g',2);
            ui->lePixelSizeXMainCCD->setText(letxt);
            letxt=QString::number((double)psy,'g',2);
            ui->lePixelSizeYMainCCD->setText(letxt);
            letxt=QString::number(fsx);
            ui->leFrameSizeXMainCCD->setText(letxt);
            letxt=QString::number(fsy);
            ui->leFrameSizeYMainCCD->setText(letxt);
            ui->lcdBitDepthMainCCD->display(bd);
        }
    }
    return retrievalSuccess;
}

 //------------------------------------------------------------------
 // store data on the guider ccd from the GUI to the global data and to the .tsp file ...
 void MainWindow::storeGuiderCCDData(void)  {
     float psx,psy;
     int ccdw, ccdh;
     QString *leEntry;

     leEntry = new QString(ui->lePixelSizeX->text());
     leEntry->replace(",",".");
     psx=leEntry->toFloat();
     leEntry->clear();
     leEntry->append(ui->lePixelSizeY->text());
     leEntry->replace(",",".");
     psy=leEntry->toFloat();
     leEntry->clear();
     leEntry->append(ui->leFrameSizeX->text());
     leEntry->replace(",",".");
     ccdw=leEntry->toInt();
     leEntry->clear();
     leEntry->append(ui->leFrameSizeY->text());
     leEntry->replace(",",".");
     ccdh=leEntry->toInt();
     delete leEntry;
     g_AllData->setCameraParameters(psx,psy,ccdw,ccdh,false);
     g_AllData->storeGlobalData();
}

 //------------------------------------------------------------------
 // store data on the guider ccd from the GUI to the global data and to the .tsp file ...
 void MainWindow::storeMainCCDData(void)  {
     float psx,psy;
     int ccdw, ccdh;
     QString *leEntry;

     leEntry = new QString(ui->lePixelSizeXMainCCD->text());
     leEntry->replace(",",".");
     psx=leEntry->toFloat();
     leEntry->clear();
     leEntry->append(ui->lePixelSizeYMainCCD->text());
     leEntry->replace(",",".");
     psy=leEntry->toFloat();
     leEntry->clear();
     leEntry->append(ui->leFrameSizeXMainCCD->text());
     leEntry->replace(",",".");
     ccdw=leEntry->toInt();
     leEntry->clear();
     leEntry->append(ui->leFrameSizeYMainCCD->text());
     leEntry->replace(",",".");
     ccdh=leEntry->toInt();
     delete leEntry;
     g_AllData->setCameraParameters(psx,psy,ccdw,ccdh,true);
     g_AllData->storeGlobalData();
 }

//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
// routines for auto guiding
//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
// correct guide star position here. called from "displayGuideCamImage".
double MainWindow::correctGuideStarPosition(float cx, float cy) {
    float devVector[2], devVectorRotated[2],errx,erry,err, devRA, devDecl;
    int pgduration;
    double aggressiveness, runningRMS, hysteresisWeight, prevWeights;
    QString logString, errString;

    hysteresisWeight = ui->sbHysteresisWeight->value(); // the weight for the last error ...
    prevWeights = (1.0-hysteresisWeight)/2.0;
    aggressiveness = ui->sbGuideAggressiveness->value(); // a value that dampens the response - values between 0.7 and 1.3
    if (this->guidingState.noOfGuidingSteps == 1) {
        ui->leDevRaPix->setText("0");
        ui->leDevDeclPix->setText("0"); 
        this->guideStarPosition.centrX = cx;
        this->guideStarPosition.centrY = cy;
        if (ui->cbLogGuidingData->isChecked()==true) {
            logString.append("Travel time per Pixel in RA in ms:\t");
            this->guidingLog->write(logString.toLatin1(),logString.length());
            logString.clear();
            logString.append(QString::number((double)this->guidingState.travelTime_ms_RA,'g',3));
            logString.append("\n");
            this->guidingLog->write(logString.toLatin1(),logString.length());
            logString.clear();
            logString.append("Travel time per Pixel in Decl in ms:\t");
            this->guidingLog->write(logString.toLatin1(),logString.length());
            logString.clear();
            logString.append(QString::number((double)this->guidingState.travelTime_ms_Decl,'g',3));
            logString.append("\n");
            this->guidingLog->write(logString.toLatin1(),logString.length());
            logString.clear();
            logString.append("Rotation matrix:\t");
            this->guidingLog->write(logString.toLatin1(),logString.length());
            logString.clear();
            logString.append(QString::number((double)this->rotMatrixGuidingXToRA[0][0],'g',3));
            logString.append("\t");
            logString.append(QString::number((double)this->rotMatrixGuidingXToRA[0][1],'g',3));
            logString.append("\n");
            this->guidingLog->write(logString.toLatin1(),logString.length());
            logString.clear();
            logString.append("\t");
            logString.append(QString::number((double)this->rotMatrixGuidingXToRA[1][0],'g',3));
            logString.append("\t");
            logString.append(QString::number((double)this->rotMatrixGuidingXToRA[1][1],'g',3));
            logString.append("\n");
            this->guidingLog->write(logString.toLatin1(),logString.length());
            logString.clear();
            logString.append("Decl backlash in ms:\t");
            this->guidingLog->write(logString.toLatin1(),logString.length());
            logString.clear();
            logString.append(QString::number((double)this->guidingState.backlashCompensationInMS,'g',3));
            logString.append("\n");
            this->guidingLog->write(logString.toLatin1(),logString.length());
            logString.clear();
            logString.append("Guidestar-position:\t");
            logString.append(QString::number((double)cx,'g',3));
            logString.append("\t");
            logString.append(QString::number((double)cy,'g',3));
            logString.append("\n");
            this->guidingLog->write(logString.toLatin1(),logString.length());
            logString.clear();
        }
        this->guidingState.rmsDevInArcSec = 0.0;
        this->guidingState.rmsDevInArcSecSum = 0.0;
        this->guidingState.maxDevInArcSec = 0;
        ui->leMaxGuideErr->setText("0/0");
        this->takeSingleGuiderCamShot();
        this->waitForNMSecs(250);
        return 0.0;
    } // when called for the first time, make the current centroid the reference ...
    if (ui->cbLogGuidingData->isChecked()==true) {
        logString.append("Measured centroid:\t");
        logString.append(QString::number((double)cx,'g',3));
        logString.append("\t");
        logString.append(QString::number((double)cy,'g',3));
        logString.append("\n");
        this->guidingLog->write(logString.toLatin1(),logString.length());
        logString.clear();
    }

    // now compute the deviation
    devVector[0]=-(this->guideStarPosition.centrX - cx);
    devVector[1]=-(this->guideStarPosition.centrY - cy); // this is the deviation in pixel from the last position
    errx=devVector[0]*this->guiding->getArcSecsPerPix(0);
    erry=devVector[1]*this->guiding->getArcSecsPerPix(1);
    if (this->guidingState.noOfGuidingSteps > 2) {
        err=sqrt(errx*errx+erry*erry);
        this->guidingState.rmsDevInArcSecSum += err*err;
        if (err > this->guidingState.maxDevInArcSec) {
            this->guidingState.maxDevInArcSec = err;
        }
        runningRMS=sqrt(1/((float)(this->guidingState.noOfGuidingSteps-1))*this->guidingState.rmsDevInArcSecSum);
        errString.clear();
        errString.append(QString::number(this->guidingState.maxDevInArcSec,'g',2));
        errString.append("/");
        errString.append(QString::number(runningRMS,'g',2));
        ui->leMaxGuideErr->setText(errString);
        if (ui->cbLogGuidingData->isChecked()==true) {
            logString.append("Current deviation:\t");
            logString.append(QString::number((double)devVector[0],'g',3));
            logString.append("\t");
            logString.append(QString::number((double)devVector[1],'g',3));
            logString.append("\n");
            this->guidingLog->write(logString.toLatin1(),logString.length());
            logString.clear();
        }
    }
    devVectorRotated[0]=(this->rotMatrixGuidingXToRA[0][0]*devVector[0]+this->rotMatrixGuidingXToRA[0][1]*devVector[1]);
    devVectorRotated[1]=(this->rotMatrixGuidingXToRA[1][0]*devVector[0]+this->rotMatrixGuidingXToRA[1][1]*devVector[1]);
    // the deviation vector is rotated to the ra/decl coordinate system and inverted as we want to move in the other direction
    if (ui->cbLogGuidingData->isChecked()==true) {
            logString.append("Transformed position:\t");
            logString.append(QString::number((double)devVectorRotated[0],'g',3));
            logString.append("\t");
            logString.append(QString::number((double)devVectorRotated[1],'g',3));
            logString.append("\n");
            this->guidingLog->write(logString.toLatin1(),logString.length());
            logString.clear();
    }

    this->guidingState.raErrs[0] = this->guidingState.raErrs[1];
    this->guidingState.raErrs[1] = this->guidingState.raErrs[2];
    this->guidingState.raErrs[2] = devVectorRotated[0]; // compute a runing average of the two past errors and the current one to dampen guiding motion
    devRA=(prevWeights*this->guidingState.raErrs[0]+prevWeights*this->guidingState.raErrs[1]+
           hysteresisWeight*this->guidingState.raErrs[2]); // this is the moving average
    // carry out the correction in RA

    if (fabs(devVectorRotated[0]) > ui->sbMaxDevInGuiding->value()) {
        pgduration=round(aggressiveness*this->guidingState.travelTime_ms_RA*fabs(devRA)); // pulse guide duration in ra
        if (pgduration > 2000) {
            pgduration = 2000;
        }
        if (ui->cbLogGuidingData->isChecked()==true) {
            logString.append("RA correction duration:\t");
            logString.append(QString::number((double)pgduration,'g',3));
            logString.append("\n");
            this->guidingLog->write(logString.toLatin1(),logString.length());
            logString.clear();
        }
        ui->lcdPulseGuideDuration->display(pgduration); // set the duration for the slew in RA - this value is used in the pulseguideroutine
        this->pulseGuideDuration=pgduration;
        ui->lePulseRAMS->setText(textEntry->number(pgduration));
        if (devRA > 0) {
            ui->leDevRaPix->setText(textEntry->number(-devRA,'g',2));
            this->raPGBwdGd(pgduration);
            if (ui->cbLogGuidingData->isChecked()==true) {
                logString.append("RA correction direction:\t RA-\n");
                this->guidingLog->write(logString.toLatin1(),logString.length());
                logString.clear();
            }
        } else {
            ui->leDevRaPix->setText(textEntry->number(devRA,'g',2));
            this->raPGFwdGd(pgduration);
            if (ui->cbLogGuidingData->isChecked()==true) {
                logString.append("RA correction direction:\t RA+\n");
                this->guidingLog->write(logString.toLatin1(),logString.length());
                logString.clear();
            }
        }
    } else {
        ui->lePulseRAMS->setText("0");
    }
    this->waitForDriveStop(true,false); // just to make sure that drive has stopped moving, should not be an issue as guiding is unthreaded
    this->waitForNMSecs(500);
    // carry out the correction in decl

    this->guidingState.declErrs[0] = this->guidingState.declErrs[1];
    this->guidingState.declErrs[1] = this->guidingState.declErrs[2];
    this->guidingState.declErrs[2] = devVectorRotated[1]; // now compute a running average for declination
    devDecl = (prevWeights*this->guidingState.declErrs[0]+prevWeights*this->guidingState.declErrs[1]+hysteresisWeight*this->guidingState.declErrs[2]);

    if (fabs(devVectorRotated[1]) > ui->sbMaxDevInGuiding->value()) {
        pgduration=round(aggressiveness*this->guidingState.travelTime_ms_Decl*fabs(devDecl)); // pulse guide duration in decl
        if (pgduration > 2000) {
            pgduration = 2000;
        }
        if (ui->cbLogGuidingData->isChecked()==true) {
                logString.append("Decl correction duration:\t");
                logString.append(QString::number((double)pgduration,'g',3));
                logString.append("\n");
                this->guidingLog->write(logString.toLatin1(),logString.length());
                logString.clear();
        }
        if (devDecl < 0) {
            if (this->guidingState.declinationDriveDirection < 0) {
                this->guidingState.declinationDriveDirection = +1; // switch state to positive travel
                if (ui->cbDeclBacklashComp->isChecked()==true) { // carry out compensation if checkbox is activated
                    this->compensateDeclBacklashPG(-this->guidingState.declinationDriveDirection); // trying to invert the correction direction ... hopefully correct
                    if (ui->cbLogGuidingData->isChecked()==true) {
                            logString.append("Decl backlash activated.\n");
                            this->guidingLog->write(logString.toLatin1(),logString.length());
                            logString.clear();
                        }
                }
                ui->lcdPulseGuideDuration->display(pgduration); // set the duration for the slew in Decl - this value is used in the pulseguideroutine
                this->pulseGuideDuration=pgduration;
            }
            ui->lePulseDeclMS->setText(textEntry->number(pgduration));
            ui->leDevDeclPix->setText(textEntry->number(-devDecl,'g',2));
            if (ui->cbSwitchDecl->isChecked() == false) {
                this->declPGMinusGd(pgduration);
            } else {
                this->declPGPlusGd(pgduration);
            }
            if (ui->cbLogGuidingData->isChecked()==true) {
                logString.append("Decl correction direction:\t Decl-\n");
                this->guidingLog->write(logString.toLatin1(),logString.length());
                logString.clear();
            }
        } else {
            if (this->guidingState.declinationDriveDirection > 0) {
                this->guidingState.declinationDriveDirection = -1; // switch state to negative travel
                if (ui->cbDeclBacklashComp->isChecked()==true) {
                    this->compensateDeclBacklashPG(-this->guidingState.declinationDriveDirection); // trying to invert the correction direction ... hopefully correct
                if (ui->cbLogGuidingData->isChecked()==true) {
                            logString.append("Decl backlash activated.\n");
                            this->guidingLog->write(logString.toLatin1(),logString.length());
                            logString.clear();
                        }
                }
                ui->lcdPulseGuideDuration->display(pgduration); // set the duration for the slew in Decl - this value is used in the pulseguideroutine
                this->pulseGuideDuration=pgduration;
            }
            ui->lePulseDeclMS->setText(textEntry->number(pgduration));
            ui->leDevDeclPix->setText(textEntry->number(devDecl,'g',2));
            if (ui->cbSwitchDecl->isChecked() == false) {
                this->declPGPlusGd(pgduration);
            } else {
                this->declPGMinusGd(pgduration);
            }
            if (ui->cbLogGuidingData->isChecked()==true) {
                logString.append("Decl correction direction:\t Decl+\n");
                this->guidingLog->write(logString.toLatin1(),logString.length());
                logString.clear();
            }
        }
    } else {
        ui->lePulseDeclMS->setText("0");
    }
    this->waitForDriveStop(false,false);
    this->takeSingleGuiderCamShot();
    this->waitForNMSecs(250);
    return 0.0;
}

//------------------------------------------------------------------
// a routine that resets the maximum guiding error
void MainWindow::resetGuidingError(void) {

    if (this->guidingState.guidingIsOn==false) {
        this->guidingState.maxDevInArcSec=0.0;
        this->guidingState.rmsDevInArcSec=0.0;
        this->guidingState.rmsDevInArcSecSum = 0.0;
        ui->leMaxGuideErr->setText(textEntry->number(this->guidingState.maxDevInArcSec));
        ui->leDevRaPix->setText("0");
        ui->leDevDeclPix->setText("0");
        ui->lePulseDeclMS->setText("0");
        ui->lePulseRAMS->setText("0");
    }
}

//------------------------------------------------------------------
// a slot that is called during calibration if the calibration process is to be stopped.
// if this variable is true, the calibration routine exits, and the value is reset
void MainWindow::terminateGuiderCalibration(void) {
    this->emergencyStop();
    this->calibrationToBeTerminated = true;
    this->displayCalibrationStatus("Termination in process...");
    QCoreApplication::processEvents(QEventLoop::AllEvents,500);
}

//------------------------------------------------------------------
// calibrate the system. the selected star is located and three
// pulse guide commands in each direction and back are carried out.
// the pixel/ms is then evaluated for each direction. 8 slews from the
// center of the search image are carried out, and the relative angle between
// a coordinate system defined by ra/decl movement and by the x/y frame
// coordinate system is defined. a log displays status messages ...
void MainWindow::calibrateAutoGuider(void) {
    int pulseDuration;
    double currentCentroid[2],initialCentroid[2], lastCentroid[2],slewVector[2],lengthOfTravel,
        travelTimeInMSForOnePixRA, travelTimeInMSForOnePixDec,
        avrgAngle, avrgDeclBacklashInPixel;
    float alpha, travelTimeInMSec;
    int thrshld,beta;
    bool medianOn, lpOn;
    short slewCounter;

    this->raState = guideTrack;
    this->deState = guideTrack;
    this->StepperDriveRA->changeMicroSteps(g_AllData->getMicroSteppingRatio(0));
    this->StepperDriveDecl->changeMicroSteps(g_AllData->getMicroSteppingRatio(0));
    this->startRATracking();
    this->guidingState.calibrationIsRunning=true;
    this->guidingState.systemIsCalibrated=false;
    ui->teCalibrationStatus->clear();
    this->calibrationToBeTerminated = false;
    ui->pbTerminateCal->setEnabled(true);
    this->displayCalibrationStatus("Entering calibration...");
    if (this->abortCCDAcquisition() == true) {
        this->displayCalibrationStatus("CCD acquisition stopped...");
    } else {
        this->displayCalibrationStatus("CCD acquisition timeout - image acquired ...");
    } // stopping the stream of images from the ccd ...

    setControlsForAutoguiderCalibration(false);
    thrshld = ui->hsThreshold->value();
    alpha = ui->hsIContrast->value()/100.0;
    beta = ui->hsIBrightness->value();
    lpOn = ui->cbLowPass->isChecked();
    medianOn=ui->cbMedianFilter->isChecked(); // get parameters for guidestar-processing from GUI
    if (this->calibrationToBeTerminated == true) {
        this->guidingState.travelTime_ms_RA=this->guidingState.travelTime_ms_Decl=100;
        this->calibrationTerminationStuffToBeDone();
        return;
    } // if the button "pbTerminateCal" is pressed, the variable "calibrationToBeTerminated" is set to true, and this function exits
    // now start a first calibration run
    this->displayCalibrationStatus("Calibration run:");
    pulseDuration = 250; // slew for two seconds
    ui->lcdPulseGuideDuration->display(pulseDuration);

    // RA calibration starts here
    initialCentroid[0] = 0;
    initialCentroid[1] = 0;
    for (slewCounter = 0; slewCounter < 4; slewCounter++) {
        this->displayCalibrationStatus("RA initial position # ",(float)slewCounter+1, "/4");
        this->waitForCalibrationImage();
        this->waitForNMSecs(250);
        this->guiding->doGuideStarImgProcessing(thrshld,medianOn,lpOn,alpha,beta,this->guidingFOVFactor,this->guidingState.guideStarSelected, true); // ... process the guide star subimage
        initialCentroid[0] += g_AllData->getInitialStarPosition(2);
        initialCentroid[1] += g_AllData->getInitialStarPosition(3); // first centroid before slew
        if (this->calibrationToBeTerminated == true) {
            this->guidingState.travelTime_ms_RA=this->guidingState.travelTime_ms_Decl=100;
            this->calibrationTerminationStuffToBeDone();
            return;
        }
        QCoreApplication::processEvents(QEventLoop::AllEvents,100);
    }
    initialCentroid[0]/=4.0;
    initialCentroid[1]/=4.0;
    currentCentroid[0] = initialCentroid[0];
    currentCentroid[1] = initialCentroid[1];
    this->displayCalibrationStatus("Initial position x: ", initialCentroid[0], "[pix]");
    this->displayCalibrationStatus("Initial position y: ", initialCentroid[1], "[pix]");
    travelTimeInMSec = 0;
    travelTimeInMSForOnePixRA = 0;
    lengthOfTravel=0;
    for (slewCounter = 0; slewCounter < 20; slewCounter++) {
        this->displayCalibrationStatus("Cal-travel in RA: ", (float)slewCounter+1, "/20");
        this->raPGFwd(); // carry out travel
        travelTimeInMSec += (float)pulseDuration;
        this->waitForDriveStop(true, false);
        this->waitForCalibrationImage();
        lastCentroid[0] = currentCentroid[0];
        lastCentroid[1] = currentCentroid[1];
        this->guiding->doGuideStarImgProcessing(thrshld,medianOn,lpOn,alpha,beta,this->guidingFOVFactor,this->guidingState.guideStarSelected, true); // ... process the guide star subimage
        currentCentroid[0] = g_AllData->getInitialStarPosition(2);
        currentCentroid[1] = g_AllData->getInitialStarPosition(3);
        slewVector[0] = currentCentroid[0]-lastCentroid[0];
        slewVector[1] = currentCentroid[1]-lastCentroid[1];  // direction vector of slew
        lengthOfTravel+=sqrt(slewVector[0]*slewVector[0]+slewVector[1]*slewVector[1]); // length of vector
        if (this->calibrationToBeTerminated == true) {
            this->guidingState.travelTime_ms_RA=this->guidingState.travelTime_ms_Decl=100;
            this->calibrationTerminationStuffToBeDone();
            return;
        }
        QCoreApplication::processEvents(QEventLoop::AllEvents,100);
    }
    travelTimeInMSForOnePixRA = lengthOfTravel/((double)travelTimeInMSec);
    this->displayCalibrationStatus("Travel for one pix in RA: ", travelTimeInMSForOnePixRA, " [ms]");
    slewVector[0] = currentCentroid[0]-initialCentroid[0];
    slewVector[1] = currentCentroid[1]-initialCentroid[1];
    avrgAngle=acos((slewVector[0])/(sqrt(slewVector[0]*slewVector[0]+slewVector[1]*slewVector[1])));
    this->displayCalibrationStatus("Rotation Angle: ", (avrgAngle*(180.0/3.14159)),"°.");
    this->rotMatrixGuidingXToRA[0][0]=cos(avrgAngle);
    this->rotMatrixGuidingXToRA[0][1]=sin(avrgAngle);
    this->rotMatrixGuidingXToRA[1][0]=-sin(avrgAngle);
    this->rotMatrixGuidingXToRA[1][1]=cos(avrgAngle);
    if (this->calibrationToBeTerminated == true) {
        this->guidingState.travelTime_ms_RA=this->guidingState.travelTime_ms_Decl=100;
        this->calibrationTerminationStuffToBeDone();
        return;
    }
    this->displayCalibrationStatus("Travel back in RA to initial position...");
    for (slewCounter = 0; slewCounter < 20; slewCounter++) {
        this->displayCalibrationStatus("Travel back in RA: ", (float)slewCounter+1, "/20");
        this->raPGBwd(); // carry out travel
        if (this->calibrationToBeTerminated == true) {
            this->guidingState.travelTime_ms_RA=this->guidingState.travelTime_ms_Decl=100;
            this->calibrationTerminationStuffToBeDone();
            return;
        }
        QCoreApplication::processEvents(QEventLoop::AllEvents,100);
    }

    this->declPGPlus(); // carry out a slew in + direction to apply tension to the worm prior to another "+" -slew
    this->waitForDriveStop(false,false);
    this->displayCalibrationStatus("Put tension on Dec-drive...");

    initialCentroid[0] = 0; // get an accurate estimate for the position again
    initialCentroid[1] = 0;
    for (slewCounter = 0; slewCounter < 4; slewCounter++) {
        this->displayCalibrationStatus("Dec initial position # ",(float)slewCounter+1, "/4");
        this->waitForCalibrationImage();
        this->waitForNMSecs(250);
        this->guiding->doGuideStarImgProcessing(thrshld,medianOn,lpOn,alpha,beta,this->guidingFOVFactor,this->guidingState.guideStarSelected, true); // ... process the guide star subimage
        initialCentroid[0] += g_AllData->getInitialStarPosition(2);
        initialCentroid[1] += g_AllData->getInitialStarPosition(3); // first centroid before slew
        if (this->calibrationToBeTerminated == true) {
            this->guidingState.travelTime_ms_RA=this->guidingState.travelTime_ms_Decl=100;
            this->calibrationTerminationStuffToBeDone();
            return;
        }
        QCoreApplication::processEvents(QEventLoop::AllEvents,100);
    }
    initialCentroid[0]/=4.0;
    initialCentroid[1]/=4.0;
    this->displayCalibrationStatus("Declination calibration");

    currentCentroid[0] = initialCentroid[0];
    currentCentroid[1] = initialCentroid[1];
    this->displayCalibrationStatus("Initial position x: ", initialCentroid[0], "[pix]");
    this->displayCalibrationStatus("Initial position y: ", initialCentroid[1], "[pix]");
    travelTimeInMSec = 0;
    travelTimeInMSForOnePixDec = 0;
    lengthOfTravel=0;
    for (slewCounter = 0; slewCounter < 20; slewCounter++) {
        this->displayCalibrationStatus("Travel in Dec: ", (float)slewCounter+1, "/20");
        this->declPGPlus(); // carry out travel
        travelTimeInMSec += (float)pulseDuration;
        this->waitForDriveStop(false, false);
        this->waitForNMSecs(250);
        this->waitForCalibrationImage();
        lastCentroid[0] = currentCentroid[0];
        lastCentroid[1] = currentCentroid[1];
        this->guiding->doGuideStarImgProcessing(thrshld,medianOn,lpOn,alpha,beta,this->guidingFOVFactor,this->guidingState.guideStarSelected, true); // ... process the guide star subimage
        currentCentroid[0] = g_AllData->getInitialStarPosition(2);
        currentCentroid[1] = g_AllData->getInitialStarPosition(3);
        slewVector[0] = currentCentroid[0]-lastCentroid[0];
        slewVector[1] = currentCentroid[1]-lastCentroid[1];  // direction vector of slew
        lengthOfTravel+=sqrt(slewVector[0]*slewVector[0]+slewVector[1]*slewVector[1]); // length of vector
        if (this->calibrationToBeTerminated == true) {
            this->guidingState.travelTime_ms_RA=this->guidingState.travelTime_ms_Decl=100;
            this->calibrationTerminationStuffToBeDone();
            return;
        }
        QCoreApplication::processEvents(QEventLoop::AllEvents,100);
    }
    travelTimeInMSForOnePixDec = lengthOfTravel/((double)travelTimeInMSec);
    this->displayCalibrationStatus("Travel for one pix in Dec: ", travelTimeInMSForOnePixDec, " [ms]");

    initialCentroid[0] = 0; // get an accurate estimate for the position again
    initialCentroid[1] = 0;
    for (slewCounter = 0; slewCounter < 4; slewCounter++) {
        this->displayCalibrationStatus("Dec position # ",(float)slewCounter+1, "/4");
        this->waitForCalibrationImage();
        this->waitForNMSecs(250);
        this->guiding->doGuideStarImgProcessing(thrshld,medianOn,lpOn,alpha,beta,this->guidingFOVFactor,this->guidingState.guideStarSelected, true); // ... process the guide star subimage
        initialCentroid[0] += g_AllData->getInitialStarPosition(2);
        initialCentroid[1] += g_AllData->getInitialStarPosition(3); // first centroid before slew
        if (this->calibrationToBeTerminated == true) {
            this->guidingState.travelTime_ms_RA=this->guidingState.travelTime_ms_Decl=100;
            this->calibrationTerminationStuffToBeDone();
            return;
        }
        QCoreApplication::processEvents(QEventLoop::AllEvents,100);
    }
    initialCentroid[0]/=4.0;
    initialCentroid[1]/=4.0;
    currentCentroid[0] = initialCentroid[0];
    currentCentroid[1] = initialCentroid[1];

    this->displayCalibrationStatus("Travel back in Decl to initial position...");
    for (slewCounter = 0; slewCounter < 20; slewCounter++) {
        this->displayCalibrationStatus("Travel back Decl: ", (float)slewCounter+1, "/20");
        this->raPGBwd(); // carry out travel
        if (this->calibrationToBeTerminated == true) {
            this->guidingState.travelTime_ms_RA=this->guidingState.travelTime_ms_Decl=100;
            this->calibrationTerminationStuffToBeDone();
            return;
        }
        QCoreApplication::processEvents(QEventLoop::AllEvents,100);
    }

    initialCentroid[0] = 0; // get an accurate estimate for the position again
    initialCentroid[1] = 0;
    for (slewCounter = 0; slewCounter < 4; slewCounter++) {
        this->displayCalibrationStatus("Dec position # ",(float)slewCounter+1, "/4");
        this->waitForCalibrationImage();
        this->waitForNMSecs(250);
        this->guiding->doGuideStarImgProcessing(thrshld,medianOn,lpOn,alpha,beta,this->guidingFOVFactor,this->guidingState.guideStarSelected, true); // ... process the guide star subimage
        initialCentroid[0] += g_AllData->getInitialStarPosition(2);
        initialCentroid[1] += g_AllData->getInitialStarPosition(3); // first centroid before slew
        if (this->calibrationToBeTerminated == true) {
            this->guidingState.travelTime_ms_RA=this->guidingState.travelTime_ms_Decl=100;
            this->calibrationTerminationStuffToBeDone();
            return;
        }
        QCoreApplication::processEvents(QEventLoop::AllEvents,100);
    }
    initialCentroid[0]/=4.0;
    initialCentroid[1]/=4.0;
    avrgDeclBacklashInPixel=sqrt((initialCentroid[0]-currentCentroid[0])*(initialCentroid[0]-currentCentroid[0])+
                                 (initialCentroid[1]-currentCentroid[1])*(initialCentroid[1]-currentCentroid[1]));

    this->guidingState.systemIsCalibrated=true; // "systemIsCalibrated" - flag set to true
    setControlsForAutoguiderCalibration(true);
    this->guidingState.rotationAngle=avrgAngle;
    this->guidingState.travelTime_ms_RA = travelTimeInMSForOnePixRA;
    this->guidingState.travelTime_ms_Decl = travelTimeInMSForOnePixDec;
    this->guidingState.backlashCompensationInMS = avrgDeclBacklashInPixel/travelTimeInMSForOnePixDec;
    this->displayCalibrationStatus("Calibration is finished...");
    this->displayCalibrationStatus("Travel time RA: ", this->guidingState.travelTime_ms_RA, "[ms/pix]");
    this->displayCalibrationStatus("Travel time Decl: ", this->guidingState.travelTime_ms_Decl, "[ms/pix]");
    ui->pbTerminateCal->setEnabled(false);
    this->calibrationToBeTerminated = false;
    ui->pbGuiding->setEnabled(true);
    this->guidingState.calibrationIsRunning=false; // "calibrationIsRunning" - flag set to false
    QCoreApplication::processEvents(QEventLoop::AllEvents,100);

}

//------------------------------------------------------------------
// a few things to be done when calibration is terminated - just to keep the code more compact
void MainWindow::calibrationTerminationStuffToBeDone(void) {

    if (this->abortCCDAcquisition() == true) {
        this->displayCalibrationStatus("CCD acquisition stopped...");
    } else {
        this->displayCalibrationStatus("CCD acquisition timeout - image acquired ...");
    } // stopping the stream of images from the ccd ...
    this->guidingState.calibrationIsRunning=false; // "calibrationIsRunning" - flag set to false
    this->guidingState.systemIsCalibrated=false; // "systemIsCalibrated" - flag set to true
    this->guidingState.rotationAngle=0.0;
    ui->pbTerminateCal->setEnabled(true);
    setControlsForAutoguiderCalibration(true);
    this->startRATracking();
    this->startCCDAcquisition(); // starting ccd acquisition again in a permanent mode ...
    this->calibrationToBeTerminated = false;
    this->displayCalibrationStatus("Calibration was terminated...");
    ui->tabGuide->setEnabled(false);
    QCoreApplication::processEvents(QEventLoop::AllEvents,100);
}

//------------------------------------------------------------------
// do a fake calibration of the system for testing purposes - called when pushing the "Skip calibration" button
void MainWindow::skipCalibration(void) {
    int pulseDuration;
    double avrgAngle;
    int imgProcWindowSize;

    ui->teCalibrationStatus->clear();
    this->displayCalibrationStatus("Skipping the calibration...");
    if (this->abortCCDAcquisition() == true) {
        this->displayCalibrationStatus("CCD acquisition stopped...");
    } else {
        this->displayCalibrationStatus("CCD acquisition timeout - image acquired ...");
    } // stopping the stream of images from the ccd ...
    this->guidingState.travelTime_ms_RA=75;
    this->guidingState.travelTime_ms_Decl=75;
    this->displayCalibrationStatus("Time for 1 pix in RA: ",this->guidingState.travelTime_ms_RA," ms");
        this->displayCalibrationStatus("Time for 1 pix in Decl: ",this->guidingState.travelTime_ms_Decl," ms");
    imgProcWindowSize=round(90*this->guidingFOVFactor*0.5); // 1/4 size of the image processing window is the travel in RA+ ...
    pulseDuration = imgProcWindowSize*this->guidingState.travelTime_ms_RA; // that gives the pulse duration
    ui->lcdPulseGuideDuration->display(pulseDuration); // set the duration for the slew
    this->pulseGuideDuration=pulseDuration;
    avrgAngle=0.0;
    this->rotMatrixGuidingXToRA[0][0]=cos(avrgAngle);
    this->rotMatrixGuidingXToRA[0][1]=sin(avrgAngle);
    this->rotMatrixGuidingXToRA[1][0]=-sin(avrgAngle);
    this->rotMatrixGuidingXToRA[1][1]=cos(avrgAngle);
    this->guidingState.backlashCompensationInMS=0;
    this->guidingState.calibrationIsRunning=false; // "calibrationIsRunning" - flag set to false
    this->guidingState.systemIsCalibrated=true; // "systemIsCalibrated" - flag set to true
    setControlsForAutoguiderCalibration(true);
    this->guidingState.rotationAngle=avrgAngle;
    this->displayCalibrationStatus("Fake calibration is finished...");
    this->waitForNMSecs(1000);
    ui->pbTerminateCal->setEnabled(false);
    this->calibrationToBeTerminated = false;
    ui->pbGuiding->setEnabled(true);
    this->guidingState.calibrationIsRunning=false; // "calibrationIsRunning" - flag set to false
    QCoreApplication::processEvents(QEventLoop::AllEvents,100);
}

//------------------------------------------------------------------
// wait until a drive has stopped. helper application for
// "calibrateAutoGuider" and "correctGuideStarPosition"
void MainWindow::waitForDriveStop(bool isRA, bool isVerbose) {

    if (isRA) {
        do {
            QCoreApplication::processEvents(QEventLoop::AllEvents,100);
        } while (this->mountMotion.RADriveIsMoving == true);
        if (isVerbose) {
            this->displayCalibrationStatus("RA motion stopped..."); // just make sure that the program continues once the drive is down
        }
    } else {
        do {
            QCoreApplication::processEvents(QEventLoop::AllEvents,100);
        } while (this->mountMotion.DeclDriveIsMoving == true);
        if (isVerbose) {
            this->displayCalibrationStatus("Decl motion stopped..."); // just make sure that the program continues once the drive is down
        }
    }
}

//------------------------------------------------------------------
// a subroutine that acquires a single image during calibration. needed in
// "calibrateAutoGuider"
void MainWindow::waitForCalibrationImage(void) {

    this->displayCalibrationStatus("Waiting for image...");
    this->guidingState.calibrationIsRunning=true;
    this->guidingState.calibrationImageReceived=false;
    this->waitForNMSecs(500);
    this->takeSingleGuiderCamShot();
    while (this->guidingState.calibrationImageReceived == false) {
        QCoreApplication::processEvents(QEventLoop::AllEvents,100);
        if (this->calibrationToBeTerminated == true) { // if the "Terminate calibration" button is pressed, this one is set to true ...
            return;
        }
    }
}

//------------------------------------------------------------------
// a subroutine that displays status messages during calibration;
// only needed to make the code more compact. this one displays
// an assembled string from values and strings; an override just gives a string.
void MainWindow::displayCalibrationStatus(QString str1, float number, QString str2) {
    QString lMesg;

    lMesg.append(str1);
    lMesg.append(QString::number((double)number,'g',3));
    lMesg.append(str2);
    ui->teCalibrationStatus->appendPlainText(lMesg);
    lMesg.clear();
}

//------------------------------------------------------------------
// an override just giving a single string message
void MainWindow::displayCalibrationStatus(QString str1) {
    ui->teCalibrationStatus->appendPlainText(str1);
}

//------------------------------------------------------------------
// resets all calibration parameters
void MainWindow::resetGuidingCalibration(void) {
    if ((this->guidingState.systemIsCalibrated==true) && (this->guidingState.guidingIsOn==false)) {
        ui->lcdGuidesteps->display(0);
        this->abortCCDAcquisition();
        ui->teCalibrationStatus->clear();
        this->guidingState.guideStarSelected=false;
        this->guidingState.guidingIsOn=false;
        g_AllData->setGuidingState(false);
        this->guidingState.calibrationIsRunning=false;
        this->guidingState.systemIsCalibrated=false;
        this->guidingState.calibrationImageReceived=false;
        this->guidingState.declinationDriveDirection=1;
        this->guidingState.travelTime_ms_RA=0.0;
        this->guidingState.travelTime_ms_Decl=0.0;
        this->guidingState.rotationAngle=0.0;
        this->guidingState.maxDevInArcSec=0.0;
        this->guidingState.rmsDevInArcSec=0.0;
        this->guidingState.rmsDevInArcSecSum = 0.0;
        this->guidingState.backlashCompensationInMS=0.0;
        this->guidingState.noOfGuidingSteps = 0;
        this->guidingState.st4IsActive=false;
        ui->pbGuiding->setEnabled(false);
        this->resetGuidingError();
        ui->tabCCDCal->setEnabled(false);
        ui->tabGuide->setEnabled(false);
        ui->tabImageProc->setEnabled(false);
        ui->tabGdContainer->setCurrentWidget(ui->tabCCDAcq);
        ui->cbAutoguiderIsCalibrated->setChecked(false);
    }
}

//------------------------------------------------------------------
// prepare the GUI and the flags for autoguiding; the actual work is done
// in "displayGuideCamImage" and "correctGuideStarPosition" ...
void MainWindow::doAutoGuiding(void) {

    if (this->guidingState.guidingIsOn == false) {

        this->raState = guideTrack;
        this->deState = guideTrack;
        this->StepperDriveRA->changeMicroSteps(g_AllData->getMicroSteppingRatio(0));
        this->StepperDriveDecl->changeMicroSteps(g_AllData->getMicroSteppingRatio(0));
        this->guidingState.raErrs[0] = this->guidingState.raErrs[1] =
        this->guidingState.raErrs[2] = 0;
        this->guidingState.declErrs[0] = this->guidingState.declErrs[1] =
        this->guidingState.declErrs[2] = 0;
        ui->rbSiderealSpeed->setChecked(true); // make sure that sidereal speed is set...
        this->setTrackingRate();
        this->startRATracking();
        this->guidingState.maxDevInArcSec=0.0;
        this->guidingState.rmsDevInArcSec=0.0;
        this->guidingState.guidingIsOn = true;
        if (ui->cbLogGuidingData->isChecked()==true) {
            this->guidingLog = new QFile("GuidingLog.tsl");
            this->guidingLog->open((QIODevice::ReadWrite | QIODevice::Text));
        }
        g_AllData->setGuidingState(this->guidingState.guidingIsOn); // this has to be known in other classes, so every "guidingIsOn" state is copied
        this->setControlsForGuiding(false);
        // take care of disabling the gui here ...
        ui->pbGuiding->setText("Stop");
        // now get a starting position and store it in the global data struct
        this->guidingState.noOfGuidingSteps = 0; // guiding starts from ground zero, so the steps are reset ...
        this->abortCCDAcquisition(); // stop the stream
        this->waitForCalibrationImage(); // wait to get a stable image
    } else {
        this->abortCCDAcquisition();
        this->guidingState.guidingIsOn = false;
        this->guidingState.calibrationIsRunning=false; // "calibrationIsRunning" - flag set to false
        if (ui->cbLogGuidingData->isChecked()==true) {
            if (this->guidingLog != NULL) {
                this->guidingLog->close();
            }
        }
        g_AllData->setGuidingState(this->guidingState.guidingIsOn); // this has to be known in other classes, so every "guidingIsOn" state is copied
        ui->pbGuiding->setText("Guide");
        this->setControlsForGuiding(true);
        this->startRATracking();
        ui->pbExpose->setEnabled(true);
        // enable the GUI here again ...
    }
}

//------------------------------------------------------------------
// slot for finding a guide star. the position is initially taken
// from the crosshair position stored in g_AllData.
void MainWindow::selectGuideStar(void) {
    int thrshld,beta;
    bool medianOn, lpOn;
    float alpha;

    if ((this->ccdGuiderCameraIsAcquiring==true) && (this->camImageWasReceived==true)) {
        if (this->mountMotion.RATrackingIsOn==false) {
            this->startRATracking();
        } // turn on tracking if it is not running when a guide star is selected
        ui->hsThreshold->setEnabled(true);
        ui->hsIContrast->setEnabled(true);
        ui->hsIBrightness->setEnabled(true); // enable image processing controls
        medianOn=ui->cbMedianFilter->isChecked();
        lpOn = ui->cbLowPass->isChecked();
        thrshld = ui->hsThreshold->value();
        alpha = ui->hsIContrast->value()/100.0;
        beta = ui->hsIBrightness->value(); // get image processing parameters
        this->guidingState.guideStarSelected=true;

        qDebug() << "doing processing...";

        this->guiding->doGuideStarImgProcessing(thrshld,medianOn,lpOn,alpha,beta,this->guidingFOVFactor,this->guidingState.guideStarSelected, true);
        this->waitForNMSecs(250);
        guideStarPosition.centrX = g_AllData->getInitialStarPosition(2);
        guideStarPosition.centrY = g_AllData->getInitialStarPosition(3); // "doGuideStarImgProcessing" stores a position in g_AllData

        qDebug() << "retrieved position now: " << guideStarPosition.centrX << "/" << g_AllData->getInitialStarPosition(3);

        qDebug() << "doing processing...";

        this->guiding->doGuideStarImgProcessing(thrshld,medianOn,lpOn,alpha,beta,this->guidingFOVFactor,this->guidingState.guideStarSelected, true);
        this->waitForNMSecs(250);
        ui->tabCCDCal->setEnabled(true);
    }
}

//------------------------------------------------------------------
// slot activated when the guide star is found; just enables the next GUI
// element; everything is just the same as in "selectGuideStar()"
void MainWindow::confirmGuideStar(void) {
    int thrshld,beta;
    bool medianOn, lpOn;
    float alpha;

    if ((this->ccdGuiderCameraIsAcquiring==true) && (this->camImageWasReceived==true)) {
        if (this->mountMotion.RATrackingIsOn==false) {
            this->startRATracking();
        } // turn on tracking if it is not running when a guide star is selected
        medianOn=ui->cbMedianFilter->isChecked();
        lpOn = ui->cbLowPass->isChecked();
        thrshld = ui->hsThreshold->value();
        alpha = ui->hsIContrast->value()/100.0;
        beta = ui->hsIBrightness->value(); // get image processing parameters
        this->guidingState.guideStarSelected=true;
        this->guiding->doGuideStarImgProcessing(thrshld,medianOn,lpOn,alpha,beta,this->guidingFOVFactor,this->guidingState.guideStarSelected, true);
        QCoreApplication::processEvents(QEventLoop::AllEvents,100);
        guideStarPosition.centrX = g_AllData->getInitialStarPosition(2);
        guideStarPosition.centrY = g_AllData->getInitialStarPosition(3); // "doGuideStarImgProcessing" stores a position in g_AllData
        this->guiding->doGuideStarImgProcessing(thrshld,medianOn,lpOn,alpha,beta,this->guidingFOVFactor,this->guidingState.guideStarSelected, true);
        ui->tabCCDCal->setEnabled(true);
        ui->pbTrainAxes->setEnabled(true);
        QCoreApplication::processEvents(QEventLoop::AllEvents,100);
    }
}

//------------------------------------------------------------------
// slot for changing the image processing controls
void MainWindow::changePrevImgProc(void) {
    int thrshld,beta;
    bool medianOn, lpOn;
    float alpha;

    thrshld = ui->hsThreshold->value();
    medianOn=ui->cbMedianFilter->isChecked();
    lpOn = ui->cbLowPass->isChecked();
    alpha = ui->hsIContrast->value()/100.0;
    beta = ui->hsIBrightness->value();
    this->guiding->doGuideStarImgProcessing(thrshld,medianOn,lpOn,alpha,beta, this->guidingFOVFactor,this->guidingState.guideStarSelected, false);
    QCoreApplication::processEvents(QEventLoop::AllEvents,100);
}

//------------------------------------------------------------------
// slot for handling a signal from the camera class that a new
// processed guide star image is available
void MainWindow::displayGuideStarPreview(void) {
    guideStarPrev = this->guiding->getGuideStarPreview();
    ui->lPreview->setPixmap(*guideStarPrev);
}

//------------------------------------------------------------------
// slot for storing the input from a spinbox on guide scope focal length
void MainWindow::changeGuideScopeFL(void) {
    int focalLength;

    focalLength=ui->sbFLGuideScope->value();
    this->guiding->setFocalLengthOfGuidescope(focalLength);
    g_AllData->setGuideScopeFocalLength(focalLength);
}

//------------------------------------------------------------------
// store guide scope focal length to .tsp preference file
void MainWindow::storeGuideScopeFL(void) {
    this->changeGuideScopeFL();
    g_AllData->storeGlobalData();
}

//------------------------------------------------------------------
// reduce the guide star window to 90x90 pixels
void MainWindow::setHalfFOV(void) {
    int thrshld,beta;
    float alpha;
    bool medianOn, lpOn;

    this->guidingFOVFactor=0.5;
    thrshld = ui->hsThreshold->value();
    alpha = ui->hsIContrast->value()/100.0;
    beta = ui->hsIBrightness->value();
    medianOn=ui->cbMedianFilter->isChecked();
    lpOn = ui->cbLowPass->isChecked();
    this->guiding->doGuideStarImgProcessing(thrshld,medianOn,lpOn,alpha,beta,this->guidingFOVFactor,this->guidingState.guideStarSelected, true);
    this->waitForNMSecs(250);
}

//------------------------------------------------------------------
// reduce the guide star window to 360x360 pixels
void MainWindow::setDoubleFOV(void) {
    int thrshld,beta;
    float alpha;
    bool medianOn, lpOn;

    this->guidingFOVFactor=2.0;
    thrshld = ui->hsThreshold->value();
    alpha = ui->hsIContrast->value()/100.0;
    beta = ui->hsIBrightness->value();
    medianOn=ui->cbMedianFilter->isChecked();
    lpOn = ui->cbLowPass->isChecked();
    this->guiding->doGuideStarImgProcessing(thrshld,medianOn,lpOn,alpha,beta,this->guidingFOVFactor,this->guidingState.guideStarSelected, true);
    this->waitForNMSecs(250);
}

//------------------------------------------------------------------
// reduce the guide star window to 180x180 pixels
void MainWindow::setRegularFOV(void) {
    int thrshld,beta;
    float alpha;
    bool medianOn, lpOn;

    this->guidingFOVFactor=1.0;
    thrshld = ui->hsThreshold->value();
    alpha = ui->hsIContrast->value()/100.0;
    beta = ui->hsIBrightness->value();
    medianOn=ui->cbMedianFilter->isChecked();
    lpOn = ui->cbLowPass->isChecked();
    this->guiding->doGuideStarImgProcessing(thrshld,medianOn,lpOn,alpha,beta,this->guidingFOVFactor,this->guidingState.guideStarSelected, true);
    this->waitForNMSecs(250);
}

//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
// TCP/IP Handbox ServerStuff
//------------------------------------------------------------------
//------------------------------------------------------------------
// a slot that handles things when an IP address for LX200 was chosen
void MainWindow::IPaddressForHandboxChosen(void) {
    QString *ipaddress;

    ipaddress = new QString(ui->listWidgetIPAddresses_2->currentItem()->text());
    g_AllData->setHandboxIPAddress(*ipaddress);
    g_AllData->storeGlobalData();
    ui->leDefaultIPAddressHBox->setText(*ipaddress);
    delete ipaddress;
    ui->pbTCPHBEnable->setEnabled(true);
}

//------------------------------------------------------------------
// a slot that establishes Handbox TCP/IP communication
void MainWindow::connectHandboxToIPSocket(void) {
    QString *ipaddress;
    qint16 hbport;

    ipaddress = new QString(g_AllData->getHandboxIPAddress()->toLatin1());
    this->HBServerAddress->setAddress(*ipaddress);
    delete ipaddress;
    hbport = 49153;
    if (this->HBServer->listen(*HBServerAddress,hbport) != true) {
    } else {
        ui->pbTCPHBEnable->setEnabled(false);
        ui->pbTCPHBDisable->setEnabled(true);
        ui->listWidgetIPAddresses_2->setEnabled(false);
    }
}

//------------------------------------------------------------------
// a slot that connects the TCP/IP handbox;
// the server listens and establishes a socket when a connection comes in
void MainWindow::establishHBIPLink(void) {
    ui->cbTCPHandboxEnabled ->setChecked(true);
    this->HBSocket = this->HBServer->nextPendingConnection();
    this->tcpHandboxIsConnected = true;
    ui->pbTCPHBKillMotion->setEnabled(true);
    ui->pbTCPHBEnable->setEnabled(false);
    ui->pbTCPHBDisable->setEnabled(true);
}

//------------------------------------------------------------------
// a slot that stops TCP/IP communication for the handbox
void MainWindow::disconnectHandboxFromIPSocket(void) {
    this->HBServer->close();
    this->HBSocket->close();
    ui->pbTCPHBEnable ->setEnabled(true);
    ui->pbTCPHBDisable->setEnabled(false);
    ui->listWidgetIPAddresses_2->setEnabled(true);
    ui->cbTCPHandboxEnabled->setChecked(false);
    ui->pbTCPHBEnable->setEnabled(true);
    ui->pbTCPHBDisable->setEnabled(false);
    this->tcpHandboxIsConnected = false;
}

//------------------------------------------------------------------
// a routine that reads data from the TCP/IP handbox
void MainWindow::readTCPHandboxData(void) {
    short charsToBeRead;

    charsToBeRead = this->HBSocket->bytesAvailable();
    if (charsToBeRead >= 10) { // the handbox sends a string of the type "xxxxxxxxxx" where x is either 0 or 1
        this->tcpHBData->clear();
        this->tcpHBData->append(this->HBSocket->readAll());
        emit tcpHandboxDataReceived();
    }
}

//------------------------------------------------------------------
// a routine that sends data to the handbox
void MainWindow::sendDataToTCPHandbox(QString dataForHandBox) {
    this->HBSocket->write(dataForHandBox.toLatin1());
}

//------------------------------------------------------------------
// a slot for the routine "sendDataToTCPHandbox"
void MainWindow::sendDataToTCPHandboxSlot(void) {
    QString *stateString;
    int gotoETA, dslrNums;

    if (this->tcpHandboxIsConnected == true) {
        stateString = new QString();
        stateString->append("RA: ");
        stateString->append(ui->leRightAscension->text());
        stateString->append("\r");
        sendDataToTCPHandbox(*stateString);
        stateString->clear();
        stateString->append("Decl: ");
        stateString->append(ui->leDecl->text());
        stateString->append("\r");
        sendDataToTCPHandbox(*stateString);
        stateString->clear();
        if ((this->mountMotion.GoToIsActiveInDecl == true) || (this->mountMotion.GoToIsActiveInRA == true)) {
            // system is in GOTO mode, send ETA to handbox
            gotoETA = (int)ui->lcdGotoTime->value();
            stateString->append("GoTo ETA: ");
            stateString->append(QString::number(gotoETA));
            stateString->append(" [s]\r");
            sendDataToTCPHandbox(*stateString);
            stateString->clear();
        }
        if (this->dslrStates.dslrSeriesRunning == true) {
            if (this->guidingState.guidingIsOn == true) {
                // send # of exposure, time elapsed and guiding error
                dslrNums = (int)ui->lcdDSLRExpsDone->value();
                stateString->append("Exp.#: ");
                stateString->append(QString::number(dslrNums));
                dslrNums = (int)(ui->lcdDSLRTimeRemaining->value());
                stateString->append("/");
                stateString->append(QString::number(dslrNums));
                stateString->append(" [s]\r");
                sendDataToTCPHandbox(*stateString);
                stateString->clear();
                stateString->append("Max/RMS: ");
                stateString->append(ui->leMaxGuideErr->text());
                stateString->append("\"\r");
                sendDataToTCPHandbox(*stateString);
                stateString->clear();
            } else {
                // send # of exposure and time elapsed
                dslrNums = (int)ui->lcdDSLRExpsDone->value();
                stateString->append("Exp.#: ");
                stateString->append(QString::number(dslrNums));
                dslrNums = (int)ui->lcdDSLRTimeRemaining->value();
                stateString->append("/: ");
                stateString->append(QString::number(dslrNums));
                stateString->append(" [s]\r");
                sendDataToTCPHandbox(*stateString);
                stateString->clear();
            }
        } else { // DSLR may be taking a single image
            if (this->dslrStates.dslrExposureIsRunning == true) {
                if (this->guidingState.guidingIsOn == true) {
                    // send time elapsed and guiding error
                    dslrNums = (int)ui->lcdDSLRTimeRemaining->value();
                    stateString->append("Exp. Time: ");
                    stateString->append(QString::number(dslrNums));
                    stateString->append(" [s]\r");
                    sendDataToTCPHandbox(*stateString);
                    stateString->clear();
                    stateString->append("Max/RMS: ");
                    stateString->append(ui->leMaxGuideErr->text());
                    stateString->append("\"\r");
                    sendDataToTCPHandbox(*stateString);
                    stateString->clear();
                } else {
                    // send # of exposure and time elapsed
                    dslrNums = (int)(ui->lcdDSLRTimeRemaining->value());
                    stateString->append("Exp. Time: ");
                    stateString->append(QString::number(dslrNums));
                    stateString->append(" [s]\r");
                    sendDataToTCPHandbox(*stateString);
                    stateString->clear();
                }
            }
        }
        if ((this->guidingState.guidingIsOn == true) && (this->dslrStates.dslrSeriesRunning == false) &&
            (this->dslrStates.dslrExposureIsRunning == false)) {
            stateString->append("Max/RMS: ");
            stateString->append(ui->leMaxGuideErr->text());
            stateString->append("\"\r");
            sendDataToTCPHandbox(*stateString);
            stateString->clear();
        }
        delete stateString;
    }
}

//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
// LX 200 related stuff
//------------------------------------------------------------------
//------------------------------------------------------------------
// a slot that handles things when an IP address for LX200 was chosen
void MainWindow::IPaddressChosen(void) {
    QString *ipaddress;

    ipaddress = new QString(ui->listWidgetIPAddresses->currentItem()->text());
    g_AllData->setLX200IPAddress(*ipaddress);
    g_AllData->storeGlobalData();
    ui->leDefaultIPAddressLX200->setText(*ipaddress);
    delete ipaddress;
    ui->pbEnableTCP->setEnabled(true);
}

//------------------------------------------------------------------
// a slot that establishes LX200 TCP/IP communication
void MainWindow::connectToIPSocket(void) {
    QString *ipaddress;
    qint16 lxport;

    ipaddress = new QString(g_AllData->getLX200IPAddress()->toLatin1());
    this->LXServerAddress->setAddress(*ipaddress);
    delete ipaddress;
    lxport = (qint16)(ui->sbLX200Port->value());
    if (this->LXServer->listen(*LXServerAddress,lxport) != true) {
    } else {
        ui->pbEnableTCP->setEnabled(false);
        ui->pbDisableTCP->setEnabled(true);
        ui->listWidgetIPAddresses->setEnabled(false);
        ui->pbLX200Active->setEnabled(false);
        ui->sbLX200Port->setEnabled(false);
    }
}

//------------------------------------------------------------------
// a slot that stops LX200 TCP/IP communication
void MainWindow::disconnectFromIPSocket(void) {
    this->LXServer->close();
    this->LXSocket->close();
    ui->pbEnableTCP->setEnabled(true);
    ui->pbDisableTCP->setEnabled(false);
    ui->listWidgetIPAddresses->setEnabled(true);
    ui->cbTCPConnected->setChecked(false);
    this->lx200IsOn=false;
    ui->pbLX200Active->setEnabled(true);
    ui->sbLX200Port->setEnabled(true);
}

//------------------------------------------------------------------
// a slot that connects a TCP/IP linked planetarium program to a socket;
// the server listens and establishes a socket when a connection comes in
void MainWindow::establishLX200IPLink(void) {
    if (this->LX200SerialPortIsUp == 0) { // if serial connection is down, then allow connecting
        ui->cbTCPConnected->setChecked(true);
        this->LXSocket = this->LXServer->nextPendingConnection();
        this->lx200IsOn=true;
    }
}

//------------------------------------------------------------------
// handle the serial port - called after timeout of the LX200Timer
void MainWindow::readLX200Port(void) {
    qint64 charsToBeRead;
    QString *command;

    if (this->lx200IsOn) {
        command = new QString();
        if (this->LX200SerialPortIsUp == 1) {
            charsToBeRead=this->lx200SerialPort->bytesAvailable();
            if (charsToBeRead > 0) {
                this->waitForNMSecs(100);
                this->lx200SerialData->clear();
                this->lx200SerialData->append(this->lx200SerialPort->readAll());
                command->append(this->lx200SerialData->data());
            }
        } else {
            charsToBeRead = this->LXSocket->bytesAvailable();
            if (charsToBeRead > 0) {
                this->waitForNMSecs(100);
                this->tcpLXdata->clear();
                this->tcpLXdata->append(this->LXSocket->readAll());
                command->append(this->tcpLXdata->data());
            }
        }
        lx200Comm->handleDataFromClient(*command);
        delete command;
    }
}

//------------------------------------------------------------------
// responding to a single <ACK> in classic LX 200 requires a statement on the
// alignment

void MainWindow::sendPolarAlignmentCommand(void) {
    if (this->LX200SerialPortIsUp == true) {
        this->lx200SerialPort->write("P");
        this->lx200SerialPort->flush();
    } else {
        this->LXSocket->write("P");
    }
}

//------------------------------------------------------------------
// if LX send RA, this slot handles it ...
void MainWindow::handleRAviaTCP(QString* racmd) {
    if (this->LX200SerialPortIsUp == true) {
        this->lx200SerialPort->write(racmd->toLatin1());
        this->lx200SerialPort->flush();
    } else {
        this->LXSocket->write(racmd->toLatin1());
    }
}

//------------------------------------------------------------------
//  if LX send decl, this slot handles it ...
void MainWindow::handleDeclviaTCP(QString* declcmd) {
    if (this->LX200SerialPortIsUp == true) {
        this->lx200SerialPort->write(declcmd->toLatin1());
        this->lx200SerialPort->flush();
    } else {
        this->LXSocket->write(declcmd->toLatin1());
    }
}

//------------------------------------------------------------------
// handles other replies by LX ...
void MainWindow::handleCommandviaTCP(QString* msgcmd) {
    if (this->LX200SerialPortIsUp == true) {
        this->lx200SerialPort->write(msgcmd->toLatin1());
        this->lx200SerialPort->flush();
    } else {
        this->LXSocket->write(msgcmd->toLatin1());
    }
}

//------------------------------------------------------------------
// log incoming requests from LX 200
void MainWindow::logLX200IncomingCmds(void) {
    QString* lx200msg;

    if ((this->lx200IsOn==true) && (ui->cbLX200Logs->isChecked()==true)) {
        lx200msg = new QString("Incoming: ");
        lx200msg->append(this->lx200Comm->getLX200Command());
        ui->teLX200Data->appendPlainText(lx200msg->toLatin1());
        delete lx200msg;
    }
    QCoreApplication::processEvents(QEventLoop::AllEvents,500);
}

//------------------------------------------------------------------
// log RA commands from LX 200
void MainWindow::logLX200OutgoingCmdsRA(void) {
    QString* lx200msg;

    if ((this->lx200IsOn==true) && (ui->cbLX200Logs->isChecked()==true)) {
        lx200msg = new QString("Outgoing RA: ");
        lx200msg->append(this->lx200Comm->getLX200ResponseRA());
        ui->teLX200Data->appendPlainText(lx200msg->toLatin1());
        delete lx200msg;
    }
    QCoreApplication::processEvents(QEventLoop::AllEvents,500);
}

//------------------------------------------------------------------
// log declination commands in LX 200
void MainWindow::logLX200OutgoingCmdsDecl(void) {
    QString* lx200msg;

    if ((this->lx200IsOn==true) && (ui->cbLX200Logs->isChecked()==true)) {
        lx200msg = new QString("Outgoing Decl: ");
        lx200msg->append(this->lx200Comm->getLX200ResponseDecl());
        ui->teLX200Data->appendPlainText(lx200msg->toLatin1());
        delete lx200msg;
    }
    QCoreApplication::processEvents(QEventLoop::AllEvents,100);
}
//------------------------------------------------------------------
// log outgoing commands from LX 200
void MainWindow::logLX200OutgoingCmds(void) {
    QString* lx200msg;

    if ((this->lx200IsOn==true) && (ui->cbLX200Logs->isChecked()==true)) {
        lx200msg = new QString("Outgoing: ");
        lx200msg->append(this->lx200Comm->getLX200Response());
        ui->teLX200Data->appendPlainText(lx200msg->toLatin1());
        delete lx200msg;
    }
    QCoreApplication::processEvents(QEventLoop::AllEvents,100);
}

//------------------------------------------------------------------
// erase the LX 200 log
void MainWindow::clearLXLog(void) {
    ui->teLX200Data->clear();
}

//------------------------------------------------------------------
// sync the mount via LX 200
void MainWindow::LXsyncMount(void) {
    QString lestr;

    if ((this->guidingState.guidingIsOn==false) && (this->guidingState.calibrationIsRunning==false)
             && (mountMotion.GoToIsActiveInDecl==false) && (mountMotion.GoToIsActiveInRA == false)) {
        if (this->StepperDriveRA->getStopped() == false) {
            this->stopRATracking();
        }
        if (this->mountMotion.DeclDriveIsMoving == true) {
            this->mountMotion.DeclDriveIsMoving=false;
            this->StepperDriveDecl->stopDrive();
        }
        this->ra = (float)(this->lx200Comm->getReceivedCoordinates(0));
        this->decl = (float)(this->lx200Comm->getReceivedCoordinates(1));
        g_AllData->setSyncPosition(this->ra, this->decl);
        this->syncMount();
        // convey right ascension and declination to the global parameters;
        // a microtimer starts ...
        this->startRATracking();
        lestr.append(this->generateCoordinateString(this->ra,true));
        ui->lineEditRA->setText(lestr);
        ui->leLX200RA->setText(lestr);
        lestr.clear();
        lestr.append(this->generateCoordinateString(this->decl,false));
        ui->lineEditDecl->setText(lestr);
        ui->leLX200Decl->setText(lestr);
        ui->pbGoTo->setEnabled(true); // enable GOTO as we now have a reference position
        ui->gbScopeParking->setEnabled(true); // enable the park position as the scope is now synced
    }
}

//---------------------------------------------------------------------
// trigger a stop via LX 200; this is not an emergency halt, it just
// terminates motion and goes into tracking state. some ASCOM drivers
// spit out :Q# like hell. I am insecure of the meaning of this command;
// in my opinion, it should stop all motion like an emergency stop, but
// the documentation says only "stop slewing motion" ...
void MainWindow::LXstopMotion(void) {

    if ((this->guidingState.guidingIsOn == false) && (this->guidingState.calibrationIsRunning == false)) {
        if ((this->mountMotion.GoToIsActiveInRA) || (this->mountMotion.GoToIsActiveInDecl) ||
            (this->mountMotion.RADriveIsMoving) || (this->mountMotion.DeclDriveIsMoving)) {
            this->emergencyStop();
            this->startRATracking();
        }
    }
}

//---------------------------------------------------------------------
// slew via LX 200
void MainWindow::LXslewMount(void) {
    QString lestr;

    if ((this->guidingState.guidingIsOn==false) && (this->guidingState.calibrationIsRunning==false)
             && (mountMotion.GoToIsActiveInDecl==false) && (mountMotion.GoToIsActiveInRA == false)) {
        if ((mountMotion.GoToIsActiveInRA==false) || (mountMotion.GoToIsActiveInDecl== false)) {
            if (g_AllData->wasMountSynced() == true) {
                QCoreApplication::processEvents(QEventLoop::AllEvents,100);
                this->ra = (float)(this->lx200Comm->getReceivedCoordinates(0));
                this->decl = (float)(this->lx200Comm->getReceivedCoordinates(1));
                lestr.append(this->generateCoordinateString(this->ra,true));
                ui->lineEditRA->setText(lestr);
                ui->leLX200RA->setText(lestr);
                lestr.clear();
                lestr.append(this->generateCoordinateString(this->decl,false));
                ui->lineEditDecl->setText(lestr);
                ui->leLX200Decl->setText(lestr);
                this->startGoToObject();
            }
        }
    }
}

//---------------------------------------------------------------------
// motion via LX 200
void MainWindow::LXmoveEast(void) {
    if ((this->guidingState.guidingIsOn==false) && (this->guidingState.calibrationIsRunning==false)
            && (mountMotion.GoToIsActiveInDecl==false) && (mountMotion.GoToIsActiveInRA == false)) {
        if ((mountMotion.GoToIsActiveInRA==false) ||
                (mountMotion.GoToIsActiveInDecl==false)) {
            if (this->mountMotion.RADriveIsMoving == true) {
                return;
            } else {
                this->RAMoveHandboxBwd();
            }
        }
    }
}

//---------------------------------------------------------------------
// motion via LX 200
void MainWindow::LXmoveWest(void) {

    if ((this->guidingState.guidingIsOn==false) && (this->guidingState.calibrationIsRunning==false)
         && (mountMotion.GoToIsActiveInDecl==false) && (mountMotion.GoToIsActiveInRA == false)) {
        if ((mountMotion.GoToIsActiveInRA==false) ||
                (mountMotion.GoToIsActiveInDecl==false)) {
            if (this->mountMotion.RADriveIsMoving == true) {
                return;
            } else {
                this->RAMoveHandboxFwd();
            }
        }
    }
}
//---------------------------------------------------------------------
// motion via LX 200
void MainWindow::LXmoveNorth(void) {
    if ((this->guidingState.guidingIsOn==false) && (this->guidingState.calibrationIsRunning==false)
             && (mountMotion.GoToIsActiveInDecl==false) && (mountMotion.GoToIsActiveInRA == false)) {
        if ((mountMotion.GoToIsActiveInRA==false) ||
                (mountMotion.GoToIsActiveInDecl==false)) {
            if (this->mountMotion.DeclDriveIsMoving == true) {
                return;
            } else {
                this->declinationMoveHandboxUp();
            }
        }
    }
}

//---------------------------------------------------------------------
// motion via LX 200
void MainWindow::LXmoveSouth(void) {
    if ((this->guidingState.guidingIsOn==false) && (this->guidingState.calibrationIsRunning==false)
             && (mountMotion.GoToIsActiveInDecl==false) && (mountMotion.GoToIsActiveInRA == false)) {
        if ((mountMotion.GoToIsActiveInRA==false) ||
                (mountMotion.GoToIsActiveInDecl==false)) {
            if (this->mountMotion.DeclDriveIsMoving == true) {
                return;
            } else {
                this->declinationMoveHandboxDown();
            }
        }
    }
}

//---------------------------------------------------------------------
// motion via LX 200
void MainWindow::LXstopMoveEast(void) {
    if ((this->guidingState.guidingIsOn==false) && (this->guidingState.calibrationIsRunning==false)
             && (mountMotion.GoToIsActiveInDecl==false) && (mountMotion.GoToIsActiveInRA == false)) {
        if (((mountMotion.GoToIsActiveInRA==false) ||
                (mountMotion.GoToIsActiveInDecl==false)) &&
                (mountMotion.RADriveIsMoving == true))  {
            this->RAMoveHandboxBwd();
        }
    }
}

//---------------------------------------------------------------------
// motion via LX 200
void MainWindow::LXstopMoveWest(void) {
    if ((this->guidingState.guidingIsOn==false) && (this->guidingState.calibrationIsRunning==false)
             && (mountMotion.GoToIsActiveInDecl==false) && (mountMotion.GoToIsActiveInRA == false)) {
        if (((mountMotion.GoToIsActiveInRA==false) ||
                (mountMotion.GoToIsActiveInDecl==false)) &&
                (mountMotion.RADriveIsMoving == true))  {
            this->RAMoveHandboxFwd();
        }
    }
}

//---------------------------------------------------------------------
// motion via LX 200
void MainWindow::LXstopMoveNorth(void) {
    if ((this->guidingState.guidingIsOn==false) && (this->guidingState.calibrationIsRunning==false)
             && (mountMotion.GoToIsActiveInDecl==false) && (mountMotion.GoToIsActiveInRA == false)) {
        if (((mountMotion.GoToIsActiveInRA==false) ||
                (mountMotion.GoToIsActiveInDecl==false)) &&
                (mountMotion.DeclDriveIsMoving == true))  {
            this->declinationMoveHandboxUp();
        }
    }
}

//---------------------------------------------------------------------
// motion via LX 200
void MainWindow::LXstopMoveSouth(void) {
    if ((this->guidingState.guidingIsOn==false) && (this->guidingState.calibrationIsRunning==false)
             && (mountMotion.GoToIsActiveInDecl==false) && (mountMotion.GoToIsActiveInRA == false)) {
        if (((mountMotion.GoToIsActiveInRA==false) ||
                (mountMotion.GoToIsActiveInDecl==false)) &&
                (mountMotion.DeclDriveIsMoving == true))  {
            this->declinationMoveHandboxDown();
        }
    }
}

//---------------------------------------------------------------------
// motion via LX 200
void MainWindow::LXslowSpeed(void) {
    if ((this->guidingState.guidingIsOn==false) && (this->guidingState.calibrationIsRunning==false)
             && (mountMotion.GoToIsActiveInDecl==false) && (mountMotion.GoToIsActiveInRA == false)) {
        ui->rbCorrSpeed->setChecked(true);
        this->setCorrectionSpeed();
    }
}

//---------------------------------------------------------------------
// motion via LX 200
void MainWindow::LXhiSpeed(void) {
    if ((this->guidingState.guidingIsOn==false) && (this->guidingState.calibrationIsRunning==false)
             && (mountMotion.GoToIsActiveInDecl==false) && (mountMotion.GoToIsActiveInRA == false)) {
        ui->rbMoveSpeed->setChecked(true);
        this->setMoveSpeed();
    }
}

//---------------------------------------------------------------------
// change LX 200 number format
void MainWindow::LXSetNumberFormatToSimple(void) {

    if (ui->cbLXSimpleNumbers->isChecked() == true) {
        this->lx200Comm->setNumberFormat(true);
    } else {
        this->lx200Comm->setNumberFormat(false);
    }
}

//---------------------------------------------------------------------
// open the serial port for LX200 communication
void MainWindow::openPort(void) {
      this->LX200SerialPortIsUp = 1;
      if (!lx200SerialPort->open(QIODevice::ReadWrite)) {
          this->LX200SerialPortIsUp = 0;
    } else {
        this->lx200SerialPort->setBreakEnabled(false);
        this->LX200SerialPortIsUp = 1;
        this->lx200SerialPort->clear(QSerialPort::AllDirections);
    }
      this->lx200Comm->clearReplyString();
}

//---------------------------------------------------------------------
// close the serial port for LX 200 commuinication
void MainWindow::shutDownPort(void) {
    this->lx200SerialPort->setBreakEnabled(true);
    this->LX200SerialPortIsUp = 0;
    this->lx200SerialPort->clear(QSerialPort::AllDirections);
    this->lx200SerialPort->close();
    this->lx200Comm->clearReplyString();
}

//---------------------------------------------------------------------
// update display of longitude, latitude and UTC offset via LX200
void MainWindow::updateLocalization(void) {
    double llong, llat;
    int lutcoffs;
    QString *dataStr;

    dataStr = new QString();
    llong = g_AllData->getSiteCoords(1);
    llat = g_AllData->getSiteCoords(0);
    lutcoffs = (int)g_AllData->getSiteCoords(2);
    dataStr->setNum(llat);
    ui->leLat->setText(dataStr->toLatin1());
    dataStr->clear();
    dataStr->setNum(llong);
    ui->leLong->setText(dataStr->toLatin1());
    ui->sbUTCOffs->setValue(lutcoffs);
    delete dataStr;
}

//---------------------------------------------------------------------
// enable or disable the serial port
void MainWindow::switchToLX200(void) {

    if ((this->lx200IsOn==false) && (this->LXSocket->isOpen() == false)) {
        this->openPort();
        if (this->LX200SerialPortIsUp == true) {
            this->lx200IsOn=true;
            ui->pbLX200Active->setText("Deactivate LX200");
            ui->cbRS232Open->setChecked(true);
            ui->tabLXTCP->setEnabled(false);
        }
    } else {
        this->shutDownPort();
        this->lx200IsOn=false;
        ui->pbLX200Active->setText("Activate LX200");
        ui->cbRS232Open->setChecked(false);
        ui->tabLXTCP->setEnabled(true);
        qDebug() << "Cannot open serial port ...";
    }
}

//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
// routines for handling the handbox
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
void MainWindow::declinationMoveHandboxUp(void) {
    long maxDeclSteps;

    if (this->mountMotion.DeclDriveIsMoving==false){
        if (ui->rbMoveSpeed->isChecked() == true) {
            this->deState = move;
        } else {
            this->deState = guideTrack;
        }
        this->StepperDriveDecl->changeMicroSteps(g_AllData->getMicroSteppingRatio((short)this->deState));
        this->mountMotion.DeclMoveElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
        ui->pbDeclDown->setEnabled(0);
        this->setControlsForDeclTravel(false);
        this->mountMotion.DeclDriveIsMoving=true;
        maxDeclSteps=180.0/g_AllData->getGearData(7 )*g_AllData->getMicroSteppingRatio((short)this->deState)*
                g_AllData->getGearData(4 )*g_AllData->getGearData(5 )*
                g_AllData->getGearData(6 ); // travel 180° at most
        this->mountMotion.DeclDriveDirection=1*g_AllData->getMFlipDecSign();
        this->StepperDriveDecl->travelForNSteps(maxDeclSteps,this->mountMotion.DeclDriveDirection*g_AllData->getMFlipDecSign(), this->mountMotion.DeclSpeedFactor,1);
    } else {
        this->mountMotion.DeclDriveIsMoving=false;
        this->deState=guideTrack;
        this->StepperDriveDecl->changeMicroSteps(g_AllData->getMicroSteppingRatio((short)this->deState));
        this->StepperDriveDecl->stopDrive();
        this->StepperDriveDecl->setInitialParamsAndComputeBaseSpeed((double)ui->sbAMaxDecl_AMIS->value(),
                                                                    ((double)(ui->sbCurrMaxDecl_AMIS->value())));
        ui->pbDeclDown->setEnabled(1);
        this->setControlsForDeclTravel(true);
        if (ui->rbMoveSpeed->isChecked()==false) {
            ui->sbMoveSpeed->setEnabled(true);
        } else {
            ui->sbMoveSpeed->setEnabled(false);
        }
        this->mountMotion.DeclDriveIsMoving=false;

        if (g_AllData->getMFlipParams(2) == true) { // a meridian flip ocurred, and upon stop, the direction needs to be switched
            g_AllData->setMFlipParams(2, false); // reset the flag that indicates a change in directions
            this->mountMotion.DeclDriveDirection *= -1*g_AllData->getMFlipDecSign();
            g_AllData->switchDeclinationSign();
        }
    }
}

//--------------------------------------------------------------
void MainWindow::declinationMoveHandboxDown(void) {
    long maxDeclSteps;

    if (this->mountMotion.DeclDriveIsMoving==false){
        if (ui->rbMoveSpeed->isChecked() == true) {
            this->deState = move;
        } else {
            this->deState = guideTrack;
        }
        this->StepperDriveDecl->changeMicroSteps(g_AllData->getMicroSteppingRatio((short)this->deState));
        this->mountMotion.DeclMoveElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
        ui->pbDeclUp->setEnabled(0);
        this->setControlsForDeclTravel(false);
        this->mountMotion.DeclDriveIsMoving=true;
        this->mountMotion.DeclDriveDirection = -1*g_AllData->getMFlipDecSign();
        maxDeclSteps=180.0/g_AllData->getGearData(7 )*g_AllData->getMicroSteppingRatio((short)this->deState) *
                g_AllData->getGearData(4 )*g_AllData->getGearData(5 )*
                g_AllData->getGearData(6 ); // travel 180° at most
        this->StepperDriveDecl->travelForNSteps(maxDeclSteps,this->mountMotion.DeclDriveDirection*g_AllData->getMFlipDecSign(),this->mountMotion.DeclSpeedFactor,1);
    } else {
        this->deState=guideTrack;
        this->StepperDriveDecl->changeMicroSteps(g_AllData->getMicroSteppingRatio((short)this->deState));
        this->StepperDriveDecl->stopDrive();
        this->StepperDriveDecl->setInitialParamsAndComputeBaseSpeed((double)ui->sbAMaxDecl_AMIS->value(),
                                                                   ((double)(ui->sbCurrMaxDecl_AMIS->value())));
        ui->pbDeclUp->setEnabled(1);
        this->setControlsForDeclTravel(true);
        if (ui->rbMoveSpeed->isChecked()==false) {
            ui->sbMoveSpeed->setEnabled(true);
        } else {
            ui->sbMoveSpeed->setEnabled(false);
        }
        this->mountMotion.DeclDriveIsMoving=false;

        if (g_AllData->getMFlipParams(2) == true) { // a meridian flip ocurred, and upon stop, the direction needs to be switched
            g_AllData->setMFlipParams(2, false); // reset the flag that indicates a change in directions
            this->mountMotion.DeclDriveDirection *= -1*g_AllData->getMFlipDecSign();
            g_AllData->switchDeclinationSign();
        }
    }
}
//--------------------------------------------------------------
void MainWindow::RAMoveHandboxFwd(void) {
    long maxRASteps;
    long fwdFactor;

    if (this->mountMotion.RATrackingIsOn == true) {
        this->stopRATracking();
        this->trackingBeforeHandboxMotionStarted = true;
    }

    ui->rbCorrSpeed->setEnabled(false);
    ui->rbMoveSpeed->setEnabled(false);
    ui->sbMoveSpeed->setEnabled(false);
    if (this->mountMotion.RADriveIsMoving == false){
        if (ui->rbMoveSpeed->isChecked() == true) {
            this->raState = move;
        } else {
            this->raState = guideTrack;
        }
        this->StepperDriveRA->changeMicroSteps(g_AllData->getMicroSteppingRatio((short)this->raState));
        this->mountMotion.RAMoveElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
        ui->pbRAMinus->setEnabled(0);
        ui->pbStartTracking->setEnabled(0);
        ui->pbStopTracking->setEnabled(0);
        this->setControlsForRATravel(false);
        this->mountMotion.RADriveIsMoving=true;
        maxRASteps=180/g_AllData->getGearData(3)*g_AllData->getMicroSteppingRatio((short)this->raState)*
                g_AllData->getGearData(0 )*g_AllData->getGearData(1 )*
                g_AllData->getGearData(2 ); // travel 180° at most

        this->mountMotion.RADriveDirection=1;
        fwdFactor = this->mountMotion.RASpeedFactor+1; // forward motion means increase the speed
        this->StepperDriveRA->travelForNSteps(maxRASteps,this->mountMotion.RADriveDirection,fwdFactor,true);
    } else {
        this->mountMotion.RADriveIsMoving=false;
        this->StepperDriveRA->stopDrive();
        this->raState=guideTrack;
        this->StepperDriveRA->changeMicroSteps(g_AllData->getMicroSteppingRatio((short)this->raState));
        this->StepperDriveRA->setInitialParamsAndComputeBaseSpeed((double)ui->sbAMaxRA_AMIS->value(),
                                                                     ((double)(ui->sbCurrMaxRA_AMIS->value())));
        if (this->mountMotion.RATrackingIsOn == false) {
            this->setControlsForRATravel(true);
            ui->pbStartTracking->setEnabled(true);
        }
        if (this->trackingBeforeHandboxMotionStarted == true) {
            this->startRATracking();
        }
        ui->pbRAMinus->setEnabled(1);
        ui->rbCorrSpeed->setEnabled(true);
        ui->rbMoveSpeed->setEnabled(true);
        if (ui->rbMoveSpeed->isChecked()==false) {
            ui->sbMoveSpeed->setEnabled(true);
        }
        this->trackingBeforeHandboxMotionStarted = false;
    }
}

//---------------------------------------------------------------------
void MainWindow::RAMoveHandboxBwd(void) {
    long maxRASteps;
    double bwdFactor;

    if (this->mountMotion.RATrackingIsOn == true) {
        this->stopRATracking();
        this->trackingBeforeHandboxMotionStarted = true;
    }

    ui->rbCorrSpeed->setEnabled(false);
    ui->rbMoveSpeed->setEnabled(false);
    ui->sbMoveSpeed->setEnabled(false);
    if (this->mountMotion.RADriveIsMoving ==false){
        if (ui->rbMoveSpeed->isChecked() == true) {
            this->raState = move;
        } else {
            this->raState = guideTrack;
        }

        this->StepperDriveRA->changeMicroSteps(g_AllData->getMicroSteppingRatio((short)this->raState));
        this->mountMotion.RAMoveElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
        ui->pbRAPlus->setEnabled(0);
        setControlsForRATravel(false);
        ui->pbStartTracking->setEnabled(0);
        ui->pbStopTracking->setEnabled(0);
        this->mountMotion.RADriveIsMoving=true;
        maxRASteps=180/g_AllData->getGearData(3)*g_AllData->getMicroSteppingRatio((short)this->raState)*
                g_AllData->getGearData(0)*g_AllData->getGearData(1 )*
                g_AllData->getGearData(2); // travel 180° at most

        this->mountMotion.RADriveDirection=-1;
        bwdFactor=this->mountMotion.RASpeedFactor-1; // backward motion means stop at tracking speeds
        this->StepperDriveRA->travelForNSteps(maxRASteps, this->mountMotion.RADriveDirection,bwdFactor,true);
    } else {
        this->mountMotion.RADriveIsMoving=false;
        this->StepperDriveRA->stopDrive();
        this->raState=guideTrack;
        this->StepperDriveRA->changeMicroSteps(g_AllData->getMicroSteppingRatio((short)this->raState));
        this->StepperDriveRA->setInitialParamsAndComputeBaseSpeed((double)ui->sbAMaxRA_AMIS->value(),
                                                                     ((double)(ui->sbCurrMaxRA_AMIS->value())));
        if (this->mountMotion.RATrackingIsOn == false) {
            this->setControlsForRATravel(true);
            ui->pbStartTracking->setEnabled(true);
        }
        if (this->trackingBeforeHandboxMotionStarted == true) {
            this->startRATracking();
        }
        ui->pbRAPlus->setEnabled(1);
        ui->rbCorrSpeed->setEnabled(true);
        ui->rbMoveSpeed->setEnabled(true);
        if (ui->rbMoveSpeed->isChecked()==false) {
            ui->sbMoveSpeed->setEnabled(true);
        }
        this->trackingBeforeHandboxMotionStarted = false;
    }
}

//--------------------------------------------------------------
// this one stops all motion and resumes tracking; is triggered when
// the "Kill Handbox Motion" button is pressed. Needed for cases where
// connection to the handbox is lost during handbox motion ...

void MainWindow::killHandBoxMotion(void) {
    this->emergencyStop();
    ui->fMainHBCtrl->setEnabled(true); // disable handcontrol widget
    ui->fHBSpeeds->setEnabled(true);
    this->mountMotion.btMoveNorth = false;
    this->mountMotion.btMoveEast = false;
    this->mountMotion.btMoveSouth = false;
    this->mountMotion.btMoveWest = false;
    this->startRATracking();
}

//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
// pulse guide routines and ST 4
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
// slot called by timeout of ST4Timer
void MainWindow::readST4Port(void) {
    if (this->guidingState.st4IsActive== true) {
        this->handleST4State();
    }
}

//--------------------------------------------------------------
// instantiates all variables for ST4
void MainWindow::startST4Guiding(void) {
    if (this->mountMotion.RATrackingIsOn==false) {
        this->startRATracking();
    } // if tracking is not active - start it ...
    this->timer->setInterval(10);
    this->StepperDriveDecl->changeMicroSteps(g_AllData->getMicroSteppingRatio(0));
    this->deState = guideTrack; // set the declination drive to guiding speed
    this->tempUpdateTimer->stop(); // no temperature updates during ST4 guiding - SPI channel 0 is only available for ST4
    ui->lcdTemp->display("-");
    this->guidingState.st4IsActive = true;
    this->guidingState.guidingIsOn = true;
    g_AllData->setGuidingState(true);
    sleep(1);
    this->st4State.nActive = false;
    this->st4State.eActive = false;
    this->st4State.sActive = false;
    this->st4State.wActive = false;
    ui->ctrlTab->setEnabled(false);
    ui->catTab->setEnabled(false);
    ui->guidingTab->setEnabled(false);
    ui->gearTab->setEnabled(false);
    ui->tabLX200->setEnabled(false);
    ui->gbHandbox ->setEnabled(false);
    ui->gbINDI->setEnabled(false);
    ui->pbStartST4->setEnabled(false);
    ui->pbStopST4->setEnabled(true);
    ui->INDITab->setEnabled(false);
    ui->cbLXSimpleNumbers->setEnabled(false);
    ui->cbLX200Logs->setEnabled(false);
    ui->pbClearLXLog->setEnabled(false);
    ui->photoTab->setEnabled(false);
    ui->locationTab->setEnabled(false);
    this->commSPIParams.guiData->clear(); // now fill the SPI queue with ST4 state requests
    this->commSPIParams.guiData->append("g");
    this->spiDrOnChan0->spidrReceiveCommand(*commSPIParams.guiData);
    this->waitForNMSecs(20);
    this->commSPIParams.guiData->clear();
    this->commSPIParams.guiData->append("g");
    this->spiDrOnChan0->spidrReceiveCommand(*commSPIParams.guiData);
    this->waitForNMSecs(20); // just make sure that nothing but ST4 responses are in the SPI register
    this->st4Timer->start(10); // if ST4 is active, the interface is read every 10 ms
    ui->lcdDEST4Lms->display(0);
    ui->lcdRAST4Lms->display(0);
    sleep(1);
}

//--------------------------------------------------------------
void MainWindow::stopST4Guiding(void) {
    this->st4Timer->stop();
    this->guidingState.st4IsActive=false;
    this->guidingState.guidingIsOn=false;
    g_AllData->setGuidingState(false);
    this->timer->setInterval(100);
    this->st4State.nActive = false;
    this->st4State.eActive = false;
    this->st4State.sActive = false;
    this->st4State.wActive = false;
    ui->ctrlTab->setEnabled(true);
    ui->catTab->setEnabled(true);
    ui->guidingTab->setEnabled(true);
    ui->INDITab->setEnabled(true);
    ui->gearTab->setEnabled(true);
    ui->tabLX200->setEnabled(true);
    ui->gbHandbox->setEnabled(true);
    ui->gbINDI->setEnabled(true);
    ui->cbLXSimpleNumbers->setEnabled(true);
    ui->photoTab->setEnabled(true);
    ui->locationTab->setEnabled(true);
    ui->cbLX200Logs->setEnabled(true);
    ui->pbClearLXLog->setEnabled(true);
    ui->pbStartST4->setEnabled(true);
    ui->pbStopST4->setEnabled(false);
    this->getTemperature(); // read the temperature sensor - it is only updated every 30 sec
    this->getTemperature(); // call it a second time to avoid trash in the SPI register
    this->waitForNMSecs(250);
    ui->cbST4North->setChecked(false); // update the GUI
    ui->cbST4East->setChecked(false);
    ui->cbST4South->setChecked(false);
    ui->cbST4West->setChecked(false);
    this->tempUpdateTimer->start(30000); // start the timer for requesting temperature again
    ui->lcdDEST4Lms->display(0);
    ui->lcdRAST4Lms->display(0);
}

//--------------------------------------------------------------
// read the state of ST4 from the Arduino Mini

void MainWindow::handleST4State(void) {
    char stateCharFromSPI;
    bool nUp, eUp, sUp, wUp;
    int raTime, deTime, currentSpeed;

    if ((this->guidingState.st4IsActive == true)) { // poll data if ST4 is active and not correcting
        this->commSPIParams.guiData->clear();
        this->commSPIParams.guiData->append("g");  // this one sets the stream of responses again to the state of ST4
        this->spiDrOnChan0->spidrReceiveCommand(*commSPIParams.guiData);
        this->waitForNMSecs(20);
        stateCharFromSPI = this->spiDrOnChan0->getResponse();
        QCoreApplication::processEvents(QEventLoop::AllEvents,500);
        switch(stateCharFromSPI) { // translate the character from the HAT arduino into a ST4 state
            case '0':   nUp=false; eUp=false; sUp=false; wUp=false;
                        break;
            case '1':   nUp=true; eUp=false; sUp=false; wUp=false;
                        break;
            case '2':   nUp=false; eUp=true; sUp=false; wUp=false;
                        break;
            case '3':   nUp=false; eUp=false; sUp=true; wUp=false;
                        break;
            case '4':   nUp=false; eUp=false; sUp=false; wUp=true;
                        break;
            case '5':   nUp=true; eUp=true; sUp=false; wUp=false;
                        break;
            case '6':   nUp=true; eUp=false; sUp=false; wUp=true;
                        break;
            case '7':   nUp=false; eUp=true; sUp=true; wUp=false;
                        break;
            case '8':   nUp=false; eUp=false; sUp=true; wUp=true;
                        break;
            default:    nUp=false; eUp=false; sUp=false; wUp=false;
                        break;
        }
        ui->cbST4North->setChecked(nUp); // update the GUI
        ui->cbST4East->setChecked(eUp);
        ui->cbST4South->setChecked(sUp);
        ui->cbST4West->setChecked(wUp);

        if (this->st4State.nActive != nUp) {
            if (nUp == true) {
                this->StepperDriveDecl->travelForNSteps(+1,(float)ui->sbGuidingRate->value());
                this->st4State.deCorrTime->start();
            } else {
                this->StepperDriveDecl->stopDrive();
                this->StepperDriveDecl->resetSteppersAfterStop();
                deTime = this->st4State.deCorrTime->elapsed();
                ui->lcdDEST4Lms->display(deTime);
            }
        }

        if (this->st4State.sActive != sUp) {
            if (sUp == true) {
                this->StepperDriveDecl->travelForNSteps(-1,(float)ui->sbGuidingRate->value());
                this->st4State.deCorrTime->start();
            } else {
                this->StepperDriveDecl->stopDrive();
                this->StepperDriveDecl->resetSteppersAfterStop();
                deTime = this->st4State.deCorrTime->elapsed();
                ui->lcdDEST4Lms->display(deTime);
            }
        }

        if (this->st4State.wActive != wUp) {
            if (wUp == true) {
                currentSpeed=this->StepperDriveRA->getKineticsFromController(3);
             //   this->StepperDriveRA->travelForNSteps(+1,(float)(1+ui->sbGuidingRate->value()));
                this->StepperDriveRA->setStepperParams((float)(currentSpeed*(1+ui->sbGuidingRate->value())),2); // alternatively, we just speed up the drive here
                qDebug() << "West is up with speed " << 1+ui->sbGuidingRate->value();
                this->st4State.raCorrTime->start();
            } else {
             //   this->StepperDriveRA->stopDrive();
             //   this->StepperDriveRA->resetSteppersAfterStop();
             //   this->StepperDriveRA->setStepperParams((float)(cu),2); // setting back to tracking speed
                this->startRATracking();
                raTime = this->st4State.raCorrTime->elapsed();
                ui->lcdRAST4Lms->display(raTime);
            }
        }

        if (this->st4State.eActive != eUp) {
            if (eUp == true) {
                currentSpeed=this->StepperDriveRA->getKineticsFromController(3);
             //   this->StepperDriveRA->travelForNSteps(+1,(float)(1-ui->sbGuidingRate->value()));
                 this->StepperDriveRA->setStepperParams((float)(currentSpeed*(1-ui->sbGuidingRate->value())),2); // alternatively, we just slow down the drive here
                qDebug() << "East is up with speed " << 1-ui->sbGuidingRate->value();
                this->st4State.raCorrTime->start();
            } else {
             //   this->StepperDriveRA->stopDrive();
             //   this->StepperDriveRA->resetSteppersAfterStop();
             //   this->StepperDriveRA->setStepperParams((float)(1),2); // setting back to tracking speed
                this->startRATracking();
                raTime = this->st4State.raCorrTime->elapsed();
                ui->lcdRAST4Lms->display(raTime);
            }
        }

        this->st4State.nActive = nUp;  // now update the ST4 states
        this->st4State.eActive = eUp;
        this->st4State.sActive = sUp;
        this->st4State.wActive = wUp;
    }
}

//--------------------------------------------------------------
void MainWindow::declPGPlus(void) {
    long duration;

    duration = this->pulseGuideDuration;
    declinationPulseGuide(duration, 1);
}

//--------------------------------------------------------------
void MainWindow::declPGMinusGd(long duration) {

    declinationPulseGuide(duration, -1);
}

//--------------------------------------------------------------
void MainWindow::declPGPlusGd(long duration) {

    declinationPulseGuide(duration, 1);
}

//--------------------------------------------------------------
void MainWindow::declPGMinus(void) {
    long duration;

    duration = this->pulseGuideDuration;
    declinationPulseGuide(duration, -1);
}
//--------------------------------------------------------------
void MainWindow::declinationPulseGuide(long pulseDurationInMS, short direction) { // to be revised
    QElapsedTimer *deTimer;

    this->setControlsForDeclTravel(false);
    ui->pbDeclDown->setEnabled(false);
    ui->pbDeclUp->setEnabled(false);
    ui->pbRAMinus->setEnabled(false);
    ui->pbRAPlus->setEnabled(false);
    if (this->mountMotion.DeclDriveIsMoving==true){
        this->mountMotion.DeclDriveIsMoving=false;
        this->StepperDriveDecl->stopDrive();
    } // if the decl drive was moving, it is now set to stop
    this->setCorrectionSpeed();
    ui->rbCorrSpeed->setChecked(true); // switch to correction speed
    this->mountMotion.DeclDriveDirection=direction*g_AllData->getMFlipDecSign();
    this->mountMotion.DeclMoveElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
    this->mountMotion.DeclDriveIsMoving=true;

    deTimer = new QElapsedTimer();
    this->StepperDriveDecl->travelForNSteps(direction,(float)ui->sbGuidingRate->value());
    deTimer->start();
    while (deTimer->elapsed() < pulseDurationInMS);
    this->StepperDriveDecl->stopDrive();
    this->StepperDriveDecl->resetSteppersAfterStop();
    delete deTimer;

    this->mountMotion.DeclDriveIsMoving=false;
    this->setControlsForDeclTravel(true);
    ui->pbDeclDown->setEnabled(true);
    ui->pbDeclUp->setEnabled(true);
    ui->pbRAMinus->setEnabled(true);
    ui->pbRAPlus->setEnabled(true);
}

//---------------------------------------------------------------------
void MainWindow::raPGFwd(void) {
    long duration;

    duration = this->pulseGuideDuration;
    raPulseGuide(duration,1);
}

//---------------------------------------------------------------------
void MainWindow::raPGBwd(void) {
    long duration;

    duration = this->pulseGuideDuration;
    raPulseGuide(duration,-1);
}

//---------------------------------------------------------------------
void MainWindow::raPGFwdGd(long duration) {

    raPulseGuide(duration,1);
}

//---------------------------------------------------------------------
void MainWindow::raPGBwdGd(long duration) {

    raPulseGuide(duration,-1);
}
//---------------------------------------------------------------------
void MainWindow::raPulseGuide(long pulseDurationInMS, short direction) { // to be revised
    QElapsedTimer *raTimer;

    this->setControlsForRATravel(false);
    ui->pbStartTracking->setEnabled(0);
    ui->pbStopTracking->setEnabled(0);
    ui->pbRAMinus->setEnabled(0);
    ui->pbRAPlus->setEnabled(0); // if the RA drive was moving, it is now set to stop
    ui->pbDeclDown->setEnabled(0);
    ui->pbDeclUp->setEnabled(0);
    this->setCorrectionSpeed();
    ui->rbCorrSpeed->setChecked(true); // switch to correction speed

    if (this->mountMotion.RATrackingIsOn) {
        this->stopRATracking();
    }
    if (this->mountMotion.RADriveIsMoving==true){
        this->mountMotion.RADriveIsMoving=false;
        this->StepperDriveRA->stopDrive();
    }
    this->mountMotion.RADriveDirection=direction;
    this->mountMotion.RAMoveElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
    this->mountMotion.RADriveIsMoving=true;

    raTimer = new QElapsedTimer();

    if (direction > 0) {
        this->StepperDriveRA->travelForNSteps(1,(float)(1+ui->sbGuidingRate->value()));
    } else {
        this->StepperDriveRA->travelForNSteps(1,(float)(1-ui->sbGuidingRate->value()));
    }
    raTimer->start();
    while (raTimer->elapsed() < pulseDurationInMS);
    this->StepperDriveRA->stopDrive();
    this->StepperDriveRA->resetSteppersAfterStop();
    delete raTimer;

    this->mountMotion.RADriveIsMoving=false;
    this->startRATracking();
    ui->pbRAMinus->setEnabled(1);
    ui->pbRAPlus->setEnabled(1);
    ui->pbDeclDown->setEnabled(1);
    ui->pbDeclUp->setEnabled(1);
    this->setControlsForRATravel(true);
}

//-----------------------------------------------------------------------
// this one is called during guiding. it compensates for declination backlash if
// the direction changes.
void MainWindow::compensateDeclBacklashPG(short ddir) { // to be revised
    long compSteps;

    if (this->guidingState.systemIsCalibrated == true) { // compensate backlash only if system was calibrated ...
        compSteps=round(g_AllData->getCelestialSpeed()*(g_AllData->getGearData(4))*
                (g_AllData->getGearData(5)*(g_AllData->getGearData(6)*
                (g_AllData->getMicroSteppingRatio(0)/(g_AllData->getGearData(7)))*
                (this->guidingState.backlashCompensationInMS/1000.0)))); // sidereal speed in declination*compensation time in s == # of steps for compensation
        this->StepperDriveDecl->travelForNSteps(compSteps, ddir, 1,false);
    }
}

//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// routines for GUI enabling and disabling
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
void MainWindow::setControlsForGuiding(bool isEnabled) {
    ui->pbTrainAxes->setEnabled(isEnabled);
    ui->pbSelectGuideStar->setEnabled(isEnabled);
    ui->sbExposureTime->setEnabled(isEnabled);
    ui->tabCCDAcq->setEnabled(isEnabled);
    ui->pbSelectGuideStar->setEnabled(isEnabled);
    ui->hsThreshold->setEnabled(isEnabled);
    ui->hsIContrast->setEnabled(isEnabled);
    ui->hsIBrightness->setEnabled(isEnabled);
    ui->cbMedianFilter->setEnabled(isEnabled);
    ui->cbLowPass->setEnabled(isEnabled);
    ui->rbFOVDbl->setEnabled(isEnabled);
    ui->rbFOVHalf->setEnabled(isEnabled);
    ui->rbFOVStd->setEnabled(isEnabled);
    ui->gearTab->setEnabled(isEnabled);
    ui->LX200Tab->setEnabled(isEnabled);
    ui->catTab->setEnabled(isEnabled);
    ui->ctrlTab->setEnabled(isEnabled);
    ui->hsThreshold->setEnabled(isEnabled);
    ui->gbAuxdrives->setEnabled(isEnabled);
    ui->sbMaxDevInGuiding->setEnabled(isEnabled);
    ui->cbDeclBacklashComp->setEnabled(isEnabled);
    ui->cbLogGuidingData->setEnabled(isEnabled);
    ui->pbResetGdErr->setEnabled(isEnabled);
    ui->pbResetGuiding->setEnabled(isEnabled);
    ui->gbCoordinates->setEnabled(isEnabled);
    ui->gbDateAndTime->setEnabled(isEnabled);
    ui->cbSwitchDecl->setEnabled(isEnabled);
    if (isEnabled == true) {
        if ((this->lx200IsOn) && (this->LXSocket->isOpen())) {
            ui->pbLX200Active->setEnabled(false);
        }
    }
    ui->tablDSLRTimer->setEnabled(true); // DSLR needs to be controlled during autoguiding
    ui->pbTerminateCal->setEnabled(false); // this one is only active when the system calibrates the autoguider
}

//---------------------------------------------------------------------
void MainWindow::setControlsForGoto(bool isEnabled) {
    ui->sbGoToSpeed->setEnabled(isEnabled);
    ui->listWidgetObject->setEnabled(isEnabled);
    ui->listWidgetCatalog->setEnabled(isEnabled);
    ui->pbSync->setEnabled(isEnabled);
    ui->pbDeclDown->setEnabled(isEnabled);
    ui->pbDeclUp->setEnabled(isEnabled);
    ui->pbRAPlus->setEnabled(isEnabled);
    ui->pbRAMinus->setEnabled(isEnabled);
    this->setControlsForDeclTravel(isEnabled);
    this->setControlsForRATravel(isEnabled);
    ui->rbCorrSpeed->setEnabled(isEnabled);
    ui->rbMoveSpeed->setEnabled(isEnabled);
    ui->sbMoveSpeed->setEnabled(isEnabled);
    ui->pbStoreDrive->setEnabled(isEnabled);
    ui->pbStoreGears->setEnabled(isEnabled);
    ui->pbLX200Active->setEnabled(isEnabled);
    ui->LX200Tab->setEnabled(isEnabled);
    ui->INDITab->setEnabled(isEnabled);
    ui->gearTab->setEnabled(isEnabled);
    ui->guidingTab->setEnabled(isEnabled);
    ui->gbCoordinates->setEnabled(isEnabled);
    ui->gbGeneralSettings->setEnabled(true);
    ui->rbSiderealSpeed->setEnabled(isEnabled);
    ui->rbLunarSpeed->setEnabled(isEnabled);
    ui->rbSolarSpeed->setEnabled(isEnabled);
    ui->cbIsOnNorthernHemisphere->setEnabled(isEnabled);
    ui->pbConveyCoordinates->setEnabled(isEnabled);
    ui->sbRAhours->setEnabled(isEnabled);
    ui->sbRAmins->setEnabled(isEnabled);
    ui->sbRASecs->setEnabled(isEnabled);
    ui->sbDeclDegrees->setEnabled(isEnabled);
    ui->sbDeclMin->setEnabled(isEnabled);
    ui->sbDeclSec->setEnabled(isEnabled);
    ui->photoTab->setEnabled(isEnabled);
    if ((isEnabled == true) && (this->auxBoardIsAvailable == false)) {
        ui->gbAuxdrives->setEnabled(false);
    }
    ui->gbCoordinates->setEnabled(isEnabled);
    ui->gbDateAndTime->setEnabled(isEnabled);
    if (isEnabled == true) {
        if ((this->lx200IsOn) && (this->LXSocket->isOpen())) {
            ui->pbLX200Active->setEnabled(false);
        }
    }
    ui->pbTerminateCal->setEnabled(false); // this one is only active when the system calibrates the autoguider
}
//---------------------------------------------------------------------
void MainWindow::setControlsForRATracking(bool isEnabled) {
    ui->sbAMaxRA_AMIS->setEnabled(isEnabled);
    ui->sbCurrMaxRA_AMIS->setEnabled(isEnabled);
    ui->leRAPlanetary->setEnabled(isEnabled);
    ui->leRAGear->setEnabled(isEnabled);
    ui->leRAWorm->setEnabled(isEnabled);
    ui->leRAStepsize->setEnabled(isEnabled);
    ui->cbIsOnNorthernHemisphere->setEnabled(isEnabled);
}

//---------------------------------------------------------------------
void MainWindow::setControlsForRATravel(bool isEnabled) {
    ui->sbAMaxRA_AMIS->setEnabled(isEnabled);
    ui->sbCurrMaxRA_AMIS->setEnabled(isEnabled);
    ui->leRAPlanetary->setEnabled(isEnabled);
    ui->leRAGear->setEnabled(isEnabled);
    ui->leRAWorm->setEnabled(isEnabled);
    ui->leRAStepsize->setEnabled(isEnabled);
    ui->cbIsOnNorthernHemisphere->setEnabled(isEnabled);
    ui->listWidgetCatalog->setEnabled(isEnabled);
    ui->listWidgetObject->setEnabled(isEnabled);
    ui->pbSync->setEnabled(isEnabled);
    ui->pbGoTo->setEnabled(isEnabled);
    ui->pbStop2->setEnabled(true);
}

//---------------------------------------------------------------------
void MainWindow::setControlsForDeclTravel(bool isEnabled) {
    ui->sbAMaxDecl_AMIS->setEnabled(isEnabled);
    ui->sbCurrMaxDecl_AMIS->setEnabled(isEnabled);
    ui->rbCorrSpeed->setEnabled(isEnabled);
    ui->rbMoveSpeed->setEnabled(isEnabled);
    ui->sbMoveSpeed->setEnabled(isEnabled);
    ui->leDeclPlanetary->setEnabled(isEnabled);
    ui->leDeclGear->setEnabled(isEnabled);
    ui->leDeclWorm->setEnabled(isEnabled);
    ui->leDeclStepSize->setEnabled(isEnabled);
    ui->listWidgetCatalog->setEnabled(isEnabled);
    ui->listWidgetObject->setEnabled(isEnabled);
    ui->pbSync->setEnabled(isEnabled);
    ui->pbGoTo->setEnabled(isEnabled);
    ui->pbStop2->setEnabled(true);
}

//---------------------------------------------------------------------
void MainWindow::setControlsForAutoguiderCalibration(bool isEnabled) {
    ui->cbAutoguiderIsCalibrated->setChecked(false);
    ui->ctrlTab->setEnabled(isEnabled);
    ui->catTab->setEnabled(isEnabled);
    ui->photoTab->setEnabled(isEnabled);
    if ((isEnabled == true) && (this->auxBoardIsAvailable == false)) {
        ui->gbAuxdrives->setEnabled(false);
    }
    ui->LX200Tab->setEnabled(isEnabled);
    this->camView->setEnabled(isEnabled);
    ui->tabCCDAcq->setEnabled(isEnabled);
    ui->tabGuide->setEnabled(isEnabled);
    ui->tabImageProc->setEnabled(isEnabled);
    ui->pbTrainAxes->setEnabled(isEnabled);
    ui->gearTab->setEnabled(isEnabled);
    ui->teCalibrationStatus->setEnabled(true);
    if (isEnabled == true) {
        if ((this->lx200IsOn) && (this->LXSocket->isOpen())) {
            ui->pbLX200Active->setEnabled(false);
        }
    }
    ui->gbCoordinates->setEnabled(isEnabled);
    ui->gbDateAndTime->setEnabled(isEnabled);
    ui->INDITab->setEnabled(isEnabled);
    ui->pbTerminateCal->setEnabled(true); // this one is only active when the system calibrates the autoguider
}

//------------------------------------------------------------------
void MainWindow::setINDIrbuttons(bool isEnabled) {
    ui->rbMoravian->setEnabled(isEnabled);
    ui->rbQHYINDI->setEnabled(isEnabled);
    ui->rbV4L2INDI->setEnabled(isEnabled);
    ui->rbZWOINDI->setEnabled(isEnabled);
    ui->rbSLXPress->setEnabled(isEnabled);
    ui->rbApogee->setEnabled(isEnabled);
    ui->rbATIK->setEnabled(isEnabled);
    ui->rbFLI->setEnabled(isEnabled);
    ui->rbINova->setEnabled(isEnabled);
    ui->rbMeadeDSI->setEnabled(isEnabled);
    ui->rbQSI->setEnabled(isEnabled);
    ui->rbSBIG->setEnabled(isEnabled);
    ui->rbToUpTek->setEnabled(isEnabled);
    ui->rbNightscape->setEnabled(isEnabled);
    ui->rbAltair->setEnabled(isEnabled);
    ui->pbTerminateCal->setEnabled(false); // this one is only active when the system calibrates the autoguider
}

//------------------------------------------------------------------
void MainWindow::setAuxDriveControls(bool isEnabled) {
    if (this->auxDriveIsStartingUp == false) {
        ui->gbAuxdrives->setEnabled(isEnabled);
        ui->gbFocuserInGuide->setEnabled(isEnabled);
        ui->pbStoreAuxData->setEnabled(isEnabled);
        ui->sbAuxAcc->setEnabled(isEnabled);
        ui->sbAuxSpeed->setEnabled(isEnabled);
        ui->pbStopAuxDrives->setEnabled(true);
    }
}

//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
// routines for handling the .tsc catalogs
//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
// this one loads a catalog according to the catalog name selected
// in the list view. catalogs have to be in a subdirectory "Catalogs"
// in the working directory.
void MainWindow::catalogChosen(QListWidgetItem* catalogName) {
    QString *catalogPath,*catName;
    long counterForObjects, maxObj;
    std::string objectName;

    ui->listWidgetCatalog->blockSignals(true);
    if (this->objCatalog != NULL) {
        delete this->objCatalog;
    }
    if (g_AllData->wasMountSynced() == true) {
        ui->pbGoTo->setEnabled(true);
    }
    ui->pbSync->setEnabled(false);
    catalogPath = new QString("Catalogs/");
    catName = new QString(catalogName->text());
    catalogPath->append(catName);
    catalogPath->append(QString(".tsc"));
    this->objCatalog = new currentObjectCatalog(*catalogPath);
    maxObj = this->objCatalog->getNumberOfObjects();
    ui->listWidgetObject->clear();
    for (counterForObjects = 0; counterForObjects < maxObj; counterForObjects++) {
        objectName=this->objCatalog->getNamesOfObjects(counterForObjects);
        ui->listWidgetObject->addItem(QString(objectName.data()));
    }
    ui->lcdCatEpoch->display(QString::number(this->objCatalog->getEpoch()));
    ui->listWidgetCatalog->blockSignals(false);
    delete catalogPath;
    delete catName;
}
//------------------------------------------------------------------
// load the catalog itself with coordinates and convert them to the current epoch
// if needed
void MainWindow::catalogObjectChosen(void) {
    QString lestr;
    long indexInList;
    double epRA, epDecl, meeusM, meeusN, deltaRA, deltaDecl,raRadians, declRadians;

    indexInList = ui->listWidgetObject->currentRow();
    if (this->objCatalog != NULL) {
        epRA=this->objCatalog->getRADec(indexInList);
        epDecl=this->objCatalog->getDeclDec(indexInList);
        if (ui->cbConvertToCurrentEpoch->isChecked()==false) {
            this->ra=epRA;
            this->decl=epDecl;
        } else {
            meeusM=(3.07234+0.00186*((ui->sbEpoch->value()-1900)/100.0))*0.00416667; // factor m, J. Meeus, 3. ed, p.63, given in degrees
            meeusN=(20.0468-0.0085*((ui->sbEpoch->value()-1900)/100.0))/(3600.0);  // factor n, in degrees
            raRadians=epRA/180.0*3.141592653589793;
            declRadians=epDecl/180.0*3.141592653589793;
            deltaRA = meeusM+meeusN*sin(raRadians)*tan(declRadians);
            deltaDecl = meeusN*cos(raRadians);
            this->ra=epRA+deltaRA*((double)(ui->sbEpoch->value()-ui->lcdCatEpoch->value()));
            if (this->ra > 360) {
                this->ra-=360;
            } // that one is clear,right - avoid more than 24h RA ...
            this->decl=epDecl+deltaDecl*((double)(ui->sbEpoch->value()-ui->lcdCatEpoch->value()));
            if (this->decl>90) {
                this->decl = 90; // don't know what else to do here
            }
            if (this->decl<-90) {
                this->decl = -90;
            }
        }
        lestr.append(this->generateCoordinateString(this->ra,true));
        ui->lineEditRA->setText(lestr);
        lestr.clear();
        lestr.append(this->generateCoordinateString(this->decl,false));
        ui->lineEditDecl->setText(lestr);
        ui->pbSync->setEnabled(true);
    }
}

//------------------------------------------------------------------
// slot that conveys manual coordinates to the controller
void MainWindow::transferCoordinates(void) {
    double rah, ram, declDeg, declMin, ras, declSec;
    double lRA, lDecl, lSubVal;
    QString lestr;

    ui->pbGoTo->setEnabled(false);
    rah = (double)(ui->sbRAhours->value());
    ram = (double)(ui->sbRAmins->value());
    ras = (double)(ui->sbRASecs->value());
    declDeg = (double)(ui->sbDeclDegrees->value());
    declMin = (double)(ui->sbDeclMin->value());
    declSec = (double)(ui->sbDeclSec->value());
    lRA = (rah + ram/60.0 + ras/3600.0)*15;
    lSubVal = (declSec/60.0+declMin)/60.0;
    if (declDeg < 0) {
        lDecl = declDeg-lSubVal;
    } else {
        lDecl = declDeg+lSubVal;
    }
    this->ra=lRA;
    this->decl=lDecl;
    lestr.append(this->generateCoordinateString(this->ra,true));
    ui->lineEditRA->setText(lestr);
    lestr.clear();
    lestr.append(this->generateCoordinateString(this->decl,false));
    ui->lineEditDecl->setText(lestr);
    ui->pbSync->setEnabled(true);
    ui->pbGoTo->setEnabled(true);
}

//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
// store drive and gear data to g_AllData and preference file
//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
void MainWindow::storeGearData(void) {
    float pgra,ogra,wormra,ssra,pgdec,ogdec,wormdec,ssdec,tmstps, mmstps, smstps,vra,vdecl;
    QString *leEntry;

    leEntry = new QString(ui->leRAPlanetary->text());
    leEntry->replace(",",".");
    pgra=leEntry->toFloat();
    leEntry->clear();
    leEntry->append(ui->leRAGear->text());
    leEntry->replace(",",".");
    ogra=leEntry->toFloat();
    leEntry->clear();
    leEntry->append(ui->leRAWorm->text());
    leEntry->replace(",",".");
    wormra=leEntry->toFloat();
    leEntry->clear();
    leEntry->append(ui->leRAStepsize->text());
    leEntry->replace(",",".");
    ssra=leEntry->toFloat();
    leEntry->clear();
    leEntry->append(ui->leDeclPlanetary->text());
    leEntry->replace(",",".");
    pgdec=leEntry->toFloat();
    leEntry->clear();
    leEntry->append(ui->leDeclGear->text());
    leEntry->replace(",",".");
    ogdec=leEntry->toFloat();
    leEntry->clear();
    leEntry->append(ui->leDeclWorm->text());
    leEntry->replace(",",".");
    wormdec=leEntry->toFloat();
    leEntry->clear();
    leEntry->append(ui->leDeclStepSize->text());
    leEntry->replace(",",".");
    ssdec=leEntry->toFloat();
    tmstps = getMStepRatios(0);
    mmstps = getMStepRatios(1);
    smstps = getMStepRatios(2);
    g_AllData->setGearData(pgra,ogra,wormra,ssra,pgdec,ogdec,wormdec,ssdec,tmstps,mmstps,smstps);
    // store all gear data in global struct
    g_AllData->storeGlobalData();

    if (this->StepperDriveRA->getStopped() == false) {
        this->stopRATracking();
        StepperDriveRA->setGearRatioAndMicrosteps(g_AllData->getGearData(0 )*g_AllData->getGearData(1 )*
                                                  g_AllData->getGearData(2 )/g_AllData->getGearData(3 ),
                                                  g_AllData->getMicroSteppingRatio(0));
        StepperDriveRA->changeSpeedForGearChange();
        this->startRATracking();
    } else {
        StepperDriveRA->setGearRatioAndMicrosteps(g_AllData->getGearData(0 )*g_AllData->getGearData(1 )*
                                                  g_AllData->getGearData(2 )/g_AllData->getGearData(3 ),
                                                  g_AllData->getMicroSteppingRatio(0));
        StepperDriveRA->changeSpeedForGearChange();
    }
    StepperDriveDecl->setGearRatioAndMicrosteps(g_AllData->getGearData(4 )*g_AllData->getGearData(5 )*
                                                g_AllData->getGearData(6 )/g_AllData->getGearData(7 ),
                                                g_AllData->getMicroSteppingRatio(0) );
    StepperDriveDecl->changeSpeedForGearChange();

    vra=StepperDriveRA->getKineticsFromController(3);
    ui->lcdVMaxRA->display(round(vra));
    vdecl=StepperDriveDecl->getKineticsFromController(3);
    ui->lcdVMaxDecl->display(round(vdecl));
    delete leEntry;
}

//---------------------------------------------------------------------
// a helper to get the right microstepping ratios
int MainWindow::getMStepRatios(short what) { // 0 for tracking/guiding, 1 for move, 2 for slewing
    int rValTrack, rValMov, rValSlew;

    if (ui->rbNormal_4_AMIS->isChecked() == true) { rValTrack = 4; }
    if (ui->rbNormal_8_AMIS->isChecked() == true) { rValTrack = 8; }
    if (ui->rbNormal_16_AMIS->isChecked() == true) { rValTrack = 16; }
    if (ui->rbNormal_32_AMIS->isChecked() == true) { rValTrack = 32; }
    if (ui->rbNormal_64_AMIS->isChecked() == true) { rValTrack = 64; }
    if (ui->rbNormal_128_AMIS->isChecked() == true) { rValTrack = 128; }
    if (ui->rbNormal_256_AMIS->isChecked() == true) { rValTrack = 156; }
    if (ui->rbMove_4_AMIS->isChecked() == true) { rValMov = 4; }
    if (ui->rbMove_8_AMIS->isChecked() == true) { rValMov = 8; }
    if (ui->rbMove_16_AMIS->isChecked() == true) { rValMov = 16; }
    if (ui->rbMove_32_AMIS->isChecked() == true) { rValMov = 32; }
    if (ui->rbMove_64_AMIS->isChecked() == true) { rValMov = 64; }
    if (ui->rbMove_128_AMIS->isChecked() == true) { rValMov = 128; }
    if (ui->rbMove_256_AMIS->isChecked() == true) { rValMov = 256; }
    if (ui->rbGoTo_4_AMIS->isChecked() == true) { rValSlew = 4; }
    if (ui->rbGoTo_8_AMIS->isChecked() == true) { rValSlew = 8; }
    if (ui->rbGoTo_16_AMIS->isChecked() == true) { rValSlew = 16; }
    if (ui->rbGoTo_32_AMIS->isChecked() == true) { rValSlew = 32; }
    if (ui->rbGoTo_64_AMIS->isChecked() == true) { rValSlew = 64; }
    if (ui->rbGoTo_128_AMIS->isChecked() == true) { rValSlew = 128; }
    if (ui->rbGoTo_256_AMIS->isChecked() == true) { rValSlew = 256; }
    switch (what) {
        case 0: return rValTrack;
        case 1: return rValMov;
        case 2: return rValSlew;
    }
    return rValTrack;
}

//---------------------------------------------------------------------
// store the drive data and convey this also to the drives
void MainWindow::storeDriveData(void) {
    g_AllData->storeGlobalData();
    this->StepperDriveRA->setStepperParams(g_AllData->getDriveParams(0,1),1);//acc
    this->StepperDriveRA->setStepperParams(g_AllData->getDriveParams(0,2),3);//current
    this->StepperDriveDecl->setStepperParams(g_AllData->getDriveParams(1,1),1);//acc
    this->StepperDriveDecl->setStepperParams(g_AllData->getDriveParams(1,2),3);//current

}

//------------------------------------------------------------------
// store DSLR pixel size and main scope focal length for dithering
void MainWindow::storeDSLRSettingsForDithering(void) {
    int ditherMinVal, ditherMaxVal;

    g_AllData->setDSLRDiagPixSize((float)(ui->sbDSLRPixSize->value()));
    g_AllData->setMainScopeFocalLength(ui->sbScopeFL->value());
    ditherMaxVal=ui->sbDitherMax->value();
    g_AllData->setDitherRange(ditherMaxVal, false);
    ditherMinVal=ui->sbDitherMin->value();
    if (ditherMinVal >= ditherMaxVal) {
        if (ditherMaxVal > 0) {
            ditherMinVal = ditherMaxVal - 1;
        } else {
            ditherMinVal = 0;
        }
        ui->sbDitherMin->setValue(ditherMinVal);
    } // just to avoid any shit
    g_AllData->setDitherRange(ditherMinVal, true);
    g_AllData->storeGlobalData();
}

//------------------------------------------------------------------
// store parameters for plate solving
void MainWindow::storeSettingsForPlateSolving(void) {
    g_AllData->setMainScopeFocalLength(ui->sbScopeFLPS->value());
    g_AllData->storeGlobalData();
    ui->sbScopeFL->setValue(g_AllData->getMainScopeFocalLength());
}

//------------------------------------------------------------------
// store data for handbox motion
void MainWindow::storeHandBoxSpeeds(void) {
    g_AllData->setHandBoxSpeeds(ui->sbGoToSpeed->value(),ui->sbMoveSpeed->value());
    g_AllData->storeGlobalData();
}

//------------------------------------------------------------------
// store data site from the GUI to the global data and to the .tsp file ...
void MainWindow::storeSiteData(void)  {
    double guilat, guilong, guiUTCOffs;
    QString *leEntry;

    leEntry = new QString(ui->leLat->text());
    leEntry->replace(",",".");
    guilat=leEntry->toDouble();
    leEntry->clear();
    leEntry->append(ui->leLong->text());
    leEntry->replace(",",".");
    guilong=leEntry->toDouble();
    leEntry->clear();
    guiUTCOffs=ui->sbUTCOffs->value();
    delete leEntry;
    g_AllData->setSiteParams(guilat,guilong,guiUTCOffs);
    g_AllData->setSiteParams(ui->leControllerName->text());
    g_AllData->storeGlobalData();
}

//----------------------------------------------------------------------
// slot that takes care of changing the tracking rates
void MainWindow::setTrackingRate(void) {
    double dracc, drcurr;

    if (ui->rbSiderealSpeed->isChecked()) {
        g_AllData->setCelestialSpeed(0);
    }
    if (ui->rbLunarSpeed->isChecked()) {
        g_AllData->setCelestialSpeed(1);
    }
    if (ui->rbSolarSpeed->isChecked()) {
        g_AllData->setCelestialSpeed(2);
    }
    dracc=this->StepperDriveRA->getKineticsFromController(2);
    drcurr=this->StepperDriveRA->getKineticsFromController(1);
    this->StepperDriveRA->setInitialParamsAndComputeBaseSpeed(dracc,drcurr);
    dracc=this->StepperDriveDecl->getKineticsFromController(2);
    drcurr=this->StepperDriveDecl->getKineticsFromController(1);
    this->StepperDriveDecl->setInitialParamsAndComputeBaseSpeed(dracc,drcurr);
}

//----------------------------------------------------------------------
// slot that responds to the strings received from the handbox  tcp/ip.
// the arduino sends a string consisting of 5 characters. "1000" is north,
// "0001" is east, "0010" is south and "0100" is west. the fifth value is 0
// if the speed is single, and 1 if the speed is the "move" speed.
// the following 5 characters control focuser motion - the selection of the 
// drive, the direction, and the stepwidth is encoded here in five digits with values 1 or 0

void MainWindow::handleHandbox(void) {
    QString *localBTCommand; // make a deep copy of the command string
    QString *dirCommand; // the first 5 characters give the directions and the speed
    QString *focuserCommand; // the next 5 characters are commands for the focuser drives
    short speedSwitchState; // set to 1 or 0 concerning the motion speed
    bool isFocuser1, isForward, CCDWasOn;
    QChar focuserValue;

    if (this->ccdGuiderCameraIsAcquiring == true) {
        CCDWasOn = true;
        this->stopCCDAcquisition();
    } else {
        CCDWasOn = false;
    } // handling motion with INDI does not really work in the event queue - so the cam is turned off
    this->waitForNMSecs(100); // just to let the transmission finish
    localBTCommand=new QString(this->tcpHBData->data()); // in this case the data comes from the TCP handbox
    dirCommand = new QString(localBTCommand->left(5));
    focuserCommand = new QString(localBTCommand->right(5));
    delete localBTCommand; // delete the local deep copy of the command string
    if ((this->guidingState.guidingIsOn==false) && (this->guidingState.calibrationIsRunning==false) &&
            (mountMotion.GoToIsActiveInDecl==false) && (mountMotion.GoToIsActiveInRA==false) &&
            (this->guidingState.st4IsActive==false)) {
        // ignore this if system is in guiding or autoguider calibration
        speedSwitchState=(dirCommand->right(1)).toInt(); // the last digit is the motion state
        dirCommand->chop(1); // remove the last character
        if (speedSwitchState == 1) {
            this->setMoveSpeed();
            ui->rbMoveSpeed->setChecked(true);
        } else {
            this->setCorrectionSpeed();
            ui->rbCorrSpeed->setChecked(true);
        } // set speeds according to the last digit
        if ((this->mountMotion.RADriveIsMoving==false) && (this->mountMotion.DeclDriveIsMoving==false)) {
            // just to make sure that the handbox does not mess up a motion initiated from the GUI
            ui->fMainHBCtrl->setEnabled(false); // disable handcontrol widget
            ui->fHBSpeeds->setEnabled(false);
            this->repaint();
            this->waitForNMSecs(500);
            if (dirCommand->compare("1000") == 0) { // start motions according the first 4 digits.
                this->mountMotion.btMoveNorth = 1;
                this->declinationMoveHandboxUp();
            }
            if (dirCommand->compare("0001") == 0) {
                this->mountMotion.btMoveWest = 1;
                this->RAMoveHandboxFwd();
            }
            if (dirCommand->compare("0010") == 0) {
                this->mountMotion.btMoveSouth = 1;
                this->declinationMoveHandboxDown();
            }
            if (dirCommand->compare("0100") == 0) {
                this->mountMotion.btMoveEast = 1;
                this->RAMoveHandboxBwd();
            }
        }
        if (dirCommand->compare("0000") == 0) {
            if (this->mountMotion.btMoveNorth == 1) {
                this->mountMotion.btMoveNorth = 0;
                this->declinationMoveHandboxUp();
            }
            if (this->mountMotion.btMoveEast == 1) {
                this->mountMotion.btMoveEast = 0;
                this->RAMoveHandboxBwd();
            }
            if (this->mountMotion.btMoveSouth == 1) {
                this->mountMotion.btMoveSouth = 0;
                this->declinationMoveHandboxDown();
            }
            if (this->mountMotion.btMoveWest == 1) {
                this->mountMotion.btMoveWest = 0;
                this->RAMoveHandboxFwd();
            }
            ui->fMainHBCtrl->setEnabled(true); // disable handcontrol widget
            ui->fHBSpeeds->setEnabled(true);
        } // stop the respective motions
    }

    if (this->auxDriveIsStartingUp == false) { // the focusercommands are only executed if the aux drives are not active
        focuserValue = focuserCommand->at(0);
        if (focuserValue.digitValue() == 1) {
            isFocuser1 = true;
        } else {
            isFocuser1 = false;
        }
        focuserValue = focuserCommand->at(1);
        if (focuserValue.digitValue() == 1) {
            isForward = true;
        } else {
            isForward = false;
        }
        focuserCommand->remove(0,2);
        if (focuserCommand->compare("100") == 0) {
            if (isForward) {
                if (isFocuser1) {
                    mvAux1FwdFullHB();
                } else {
                    mvAux2FwdFullHB();
                }
            } else {
                if (isFocuser1) {
                    mvAux1BwdFullHB();
                } else {
                    mvAux2BwdFullHB();
                }
            }
        } else if (focuserCommand->compare("010") == 0) {
            if (isForward) {
                if (isFocuser1) {
                    mvAux1FwdSmallHB();
                } else {
                    mvAux2FwdSmallHB();
                }
            } else {
                if (isFocuser1) {
                    mvAux1BwdSmallHB();
                } else {
                    mvAux2BwdSmallHB();
                }
            }
        } else if (focuserCommand->compare("001") == 0) {
            if (isForward) {
                if (isFocuser1) {
                    mvAux1FwdTinyHB();
                } else {
                    mvAux2FwdTinyHB();
                }
            } else {
                if (isFocuser1) {
                    mvAux1BwdTinyHB();
                } else {
                    mvAux2BwdTinyHB();
                }
            }
        }
    } // decoded the 5 integers indicating the number of the focuser, the direction, and the amount of the step
    delete dirCommand; // delete the string containing the first 5 characters for handbox control
    delete focuserCommand; // delete the last 5 characters for the focuser
    if (CCDWasOn == true) {
        this->startCCDAcquisition();
    } // if the ccd was stopped - start it again
}

//-----------------------------------------------------------------
//-----------------------------------------------------------------
//-----------------------------------------------------------------
// series of methods related to the DSLR ...
//-----------------------------------------------------------------
//-----------------------------------------------------------------
//-----------------------------------------------------------------
// slot that handles start of DSLR exposure ...
void MainWindow::handleDSLRSingleExposure(void) {
    int duration;

    duration = ui->sbDSLRDuration->value()+0.25;
    dslrStates.dslrExpTime=duration;
    dslrStates.dslrExposureIsRunning=true;
    dslrStates.dslrExpElapsed.start();
    ui->pbDSLRSingleShot->setEnabled(false);
    ui->sbDSLRDuration->setEnabled(false);
    ui->pbDSLRStartSeries->setEnabled(false);
    if (this->dslrStates.dslrSeriesRunning == false) {
        ui->pbDSLRTerminateExposure->setEnabled(true);
    }
    digitalWrite(1,1); // set wiring pi pin 1=tip to high ... start exposure
    this->waitForNMSecs(250);
}

//------------------------------------------------------------------
// slot that handles updates of the GUI during DSLR exposure - i.e.
// display of the remaining exposure time. it emits a signal once
// the exposure is done. this may trigger another exposure if a series
// is carried out ...
void MainWindow::updateDSLRGUIAndCountdown(void) {
    int remTime;

    remTime = (this->dslrStates.dslrExpTime - round(this->dslrStates.dslrExpElapsed.elapsed()/1000.0));
    ui->lcdDSLRTimeRemaining->display(QString::number(remTime));
    if (this->dslrStates.dslrExpElapsed.elapsed() > this->ui->sbDSLRDuration->value()*1000) {
        digitalWrite(1,0); // set wiring pi pin 1=tip/expose to low ...
        this->waitForNMSecs(250);
        ui->pbDSLRSingleShot->setEnabled(true);
        if (this->dslrStates.dslrSeriesRunning == false) {
            ui->pbDSLRTerminateExposure->setEnabled(false);
        }
        ui->sbDSLRDuration->setEnabled(true);
        this->dslrStates.dslrExposureIsRunning = false;
        ui->pbDSLRStartSeries->setEnabled(true);
        if (this->dslrStates.dslrSeriesRunning == true) {
            emit dslrExposureDone();
        }
    }
}

//------------------------------------------------------------------
// slot that starts a series of exposures ...
void MainWindow::startDSLRSeries(void) {

    if (ui->sbDSLRRepetitions->value() > 0) {
        qsrand((uint)(UTTime->currentTime().second())); // initialize the random number generator
        ui->cbExpSeriesDone->setChecked(false);
        ui->sbDSLRRepetitions->setEnabled(false);
        ui->pbDSLRStartSeries->setEnabled(false);
        ui->pbDSLRStopSeries->setEnabled(true);
        ui->pbDSLRTerminateExposure->setEnabled(false);
        ui->cbDither->setEnabled(false);
        ui->lcdTempSeriesDiff->display(0);
        this->dslrStates.dslrSeriesRunning = true;
        this->dslrStates.noOfExposures = ui->sbDSLRRepetitions->value();
        this->dslrStates.noOfExposuresLeft=this->dslrStates.noOfExposures;
        this->dslrStates.tempAtSeriesStart=this->temperature;
        ui->sbDSLRRepetitions->setEnabled(false);
        ui->lcdDSLRExpsDone->display("1");
        this->dslrStates.dslrExpElapsed.restart();
        this->handleDSLRSingleExposure();
        ui->sbPauseBetweenExpSeries->setEnabled(false);
    }
}

//-------------------------------------------------------------------
// slot that is called once an exposure was taken; also takes care of dithering if checkbox is set
void MainWindow::takeNextExposureInSeries(void) {
    QElapsedTimer *wait;
    int expsTaken, msecondsRemaining;
    bool waitMore = false;
    bool inGuiding = false;
    float tempDiff; // difference between temperature at series start and next exposure
    long pauseBetweenExpsInMS = 5000; // time as read from the GUI between single exposures

    if (this->dslrStates.noOfExposuresLeft > 1) {
        this->getTemperature(); // read the temperature sensor
        tempDiff = this->temperature - this->dslrStates.tempAtSeriesStart;
        ui->lcdTempSeriesDiff->display(round(tempDiff));
        ui->pbDSLRSingleShot->setEnabled(false);
        ui->sbDSLRDuration->setEnabled(false);
        ui->pbDSLRStartSeries->setEnabled(false);
        ui->cbDither->setEnabled(false);
        this->dslrStates.noOfExposuresLeft--;
        expsTaken=this->dslrStates.noOfExposures-this->dslrStates.noOfExposuresLeft;

        if (this->guidingState.guidingIsOn == true) {
            inGuiding = true; // dithering turns guiding off; after that, the routine waits until guiding is on again.
            // this variable keeps track whether guiding was on ...
        }
        if (inGuiding == true) {
            this->carryOutDitheringStep();
            do {
                this->waitForNMSecs(1000);
            } while (this->guidingState.guidingIsOn == false);
            // wait until guiding starts again ...
            do {
                this->waitForNMSecs(1000);
            } while (this->guidingState.noOfGuidingSteps < 4); // wait for guiding to stabilize
        }
        wait = new QElapsedTimer();
        wait->start(); // start the timer for waiting between exposures ...
        if (wait->elapsed() < pauseBetweenExpsInMS) {
            msecondsRemaining = pauseBetweenExpsInMS-wait->elapsed();
            waitMore = true; // just wait for a given # of seconds until next exposure is taken
        }
        if (waitMore == true) {
            this->waitForNMSecs(msecondsRemaining);
        }
        delete wait;
        if (this->dslrStates.noOfExposuresLeft > 0) { // it is possible to terminate the series in a pause ... this has to be taken care of ...
            this->handleDSLRSingleExposure();
            ui->lcdDSLRExpsDone->display(QString::number(expsTaken+1));
        } else {
            this->stopDSLRExposureSeries();
            this->waitForNMSecs(500);
        }
        if (this->dslrStates.noOfExposuresLeft == 0) {
            this->stopDSLRExposureSeries();
            this->waitForNMSecs(500);
        }
    } else {
        this->stopDSLRExposureSeries();
        this->waitForNMSecs(500);
    }
}

//-------------------------------------------------------------------
// this routine is called in a series - it does dithering of DSLR exposures

void MainWindow::carryOutDitheringStep(void) {
    int randDeclPixStep, randRAPixStep, pixDisplacement;
    float invRandMax = 1/(float)RAND_MAX;
    float travelTimeMS, raTimeMS, declTimeMS, guideCamVsDSLRRatio, avrgGuiderPixSize;;
    short sign;
    bool restartGuiding = false;

    if (ui->cbDither->isChecked() == true) { // this is the dithering routine
        if (this->guidingState.guidingIsOn == true) {
            this->doAutoGuiding();
            restartGuiding = true;
        } // stop autoguiding
        raTimeMS = -this->dslrStates.ditherTravelInMSRA;
        declTimeMS = -this->dslrStates.ditherTravelInMSDecl; // read the last dither step times and revert these
        if (raTimeMS < 0) {
            this->raPGBwdGd(abs(raTimeMS));
        } else {
            this->raPGFwdGd(abs(raTimeMS));
        }
        this->waitForNMSecs(250);
        if (declTimeMS < 0) {
            this->declPGMinusGd(abs(declTimeMS));
        } else {
            this->declPGPlusGd(abs(declTimeMS));
        } // went back to old position before last dither step ...
        this->waitForNMSecs(250);
        avrgGuiderPixSize=sqrt((g_AllData->getCameraPixelSize(0,false))*(g_AllData->getCameraPixelSize(0,false)) +
                            (g_AllData->getCameraPixelSize(1,false)*(g_AllData->getCameraPixelSize(1,false))));
        guideCamVsDSLRRatio=(avrgGuiderPixSize*g_AllData->getMainScopeFocalLength())/
                (g_AllData->getDSLRDiagPixSize()*g_AllData->getGuideScopeFocalLength());
        // correction times for guiding are scaled for the guiding scope, but here they are used
        // guiding scope, but here they are used for them main scope, therefore this factor is necessary
        do {
            randRAPixStep = qrand()*invRandMax*g_AllData->getDitherRange(false);
            randDeclPixStep = qrand()*invRandMax*g_AllData->getDitherRange(false);
            pixDisplacement = round(sqrt(randRAPixStep*randRAPixStep+randDeclPixStep*randDeclPixStep));
        } while ((pixDisplacement < g_AllData->getDitherRange(true)) || (pixDisplacement > g_AllData->getDitherRange(false)));
        ui->lcdDitherStep->display(pixDisplacement);
        if (this->guidingState.travelTime_ms_RA < 0.1) {
            travelTimeMS = 100; // just set something if nothing is set
        } else {
            travelTimeMS = this->guidingState.travelTime_ms_RA;
        }
        travelTimeMS*=guideCamVsDSLRRatio;
        sign = 1;
        if (qrand()%2 == 0) {
            sign = -1;
        } // compute a random sign for the first displacement
        raTimeMS = randRAPixStep*travelTimeMS;
        if (qrand()%2 == 0) {
            sign *= -1;
        } // compute a random sign for the second displacement
        declTimeMS = randDeclPixStep*travelTimeMS;
        if (raTimeMS > 2000) {
            raTimeMS=2000;
        }
        if (declTimeMS > 2000) {
            declTimeMS=2000;
        } // limit travel time if something goes wrong
        this->dslrStates.ditherTravelInMSRA = raTimeMS;
        this->dslrStates.ditherTravelInMSDecl = declTimeMS; // remember the steps
        if (raTimeMS < 0) {
            this->raPGBwdGd(abs(raTimeMS));
        } else {
            this->raPGFwdGd(abs(raTimeMS));
        }
        this->waitForNMSecs(250);
        if (declTimeMS < 0) {
            this->declPGMinusGd(abs(declTimeMS));
        } else {
            this->declPGPlusGd(abs(declTimeMS));
        } // did dither step ...
        this->waitForNMSecs(250);
        if ((this->guidingState.systemIsCalibrated == true) && (restartGuiding == true)) {
            this->doAutoGuiding();
            // start autoguiding
        }
    }
}

//-------------------------------------------------------------------
// this slot is called when all exposures of a series are taken
void MainWindow::stopDSLRExposureSeries(void) {

    if (this->dslrStates.dslrExposureIsRunning == true) {
        this->terminateDSLRSingleShot(); // if an exposure is running - terminate it ...
    } else {
        dslrStates.dslrExpElapsed.invalidate(); // otherwise just kill the timer for the series ...
    }
    this->dslrStates.ditherTravelInMSRA = 0;
    this->dslrStates.ditherTravelInMSDecl = 0;
    this->dslrStates.dslrSeriesRunning = false;
    ui->sbPauseBetweenExpSeries->setEnabled(true);
    ui->pbDSLRSingleShot->setEnabled(true);
    ui->sbDSLRDuration->setEnabled(true);
    ui->cbExpSeriesDone->setChecked(true);
    ui->sbDSLRRepetitions->setEnabled(true);
    ui->pbDSLRStartSeries->setEnabled(true);
    ui->cbDither->setEnabled(true);
    ui->pbDSLRStopSeries->setEnabled(false);
    ui->sbDSLRRepetitions->setValue(0);
    ui->lcdDitherStep->display(0);
    ui->lcdDSLRExpsDone->display(0);

}

//--------------------------------------------------------------------
// this slot terminates a series of exposures prematurely
void MainWindow::terminateDSLRSeries(void) {

    ui->sbPauseBetweenExpSeries->setEnabled(true);
    this->dslrStates.dslrSeriesRunning = false;
    this->dslrStates.noOfExposuresLeft = 0;
    this->stopDSLRExposureSeries();
    this->dslrStates.noOfExposuresLeft = 0;
    this->terminateDSLRSingleShot();
    ui->lcdDSLRExpsDone->display(0);
    this->dslrStates.ditherTravelInMSRA = 0;
    this->dslrStates.ditherTravelInMSDecl = 0;
    ui->lcdDitherStep->display(0);
}

//--------------------------------------------------------------------
// this slot stops a single exposure
void MainWindow::terminateDSLRSingleShot(void) {
    this->dslrStates.dslrExposureIsRunning=false;
    this->dslrStates.dslrExpElapsed.invalidate();
    ui->pbDSLRSingleShot->setEnabled(true);
    ui->pbDSLRTerminateExposure->setEnabled(false);
    ui->sbDSLRDuration->setEnabled(true);
    ui->pbDSLRStartSeries->setEnabled(true);
    ui->lcdDSLRTimeRemaining->display("0");
    digitalWrite(1,0); // set wiring pi pin 1=tip/expose to low ...
    this->waitForNMSecs(250);
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
//---------------------------------------------------------------------
// routine for polling temperature from the HAT arduino

void MainWindow::getTemperature(void) {
    QString temp;
    float lTmp = 0;
    char lastC;

    this->commSPIParams.guiData->clear();
    this->commSPIParams.guiData->append("p");
    this->spiDrOnChan0->spidrReceiveCommand(*commSPIParams.guiData);
    this->waitForNMSecs(25);
    if (this->spiDrOnChan0->getResponse() == 'p') {
        this->commSPIParams.guiData->clear();
        this->commSPIParams.guiData->append("p");
        this->spiDrOnChan0->spidrReceiveCommand(*commSPIParams.guiData);
        this->waitForNMSecs(25);
    } // if the arduino is to slow, one gets back the command - then the call is repeated ...
    this->commSPIParams.guiData->clear();
    this->commSPIParams.guiData->append("b");
    this->spiDrOnChan0->spidrReceiveCommand(*commSPIParams.guiData);
    this->waitForNMSecs(25);
    if (this->spiDrOnChan0->getResponse() == 'b') {
        this->commSPIParams.guiData->clear();
        this->commSPIParams.guiData->append("b");
        this->spiDrOnChan0->spidrReceiveCommand(*commSPIParams.guiData);
        this->waitForNMSecs(25);
    }
    temp.append(this->spiDrOnChan0->getResponse());
    this->commSPIParams.guiData->clear();
    this->commSPIParams.guiData->append("l");
    this->spiDrOnChan0->spidrReceiveCommand(*commSPIParams.guiData);
    this->waitForNMSecs(25);
    if (this->spiDrOnChan0->getResponse() == 'l') {
        this->commSPIParams.guiData->clear();
        this->commSPIParams.guiData->append("l");
        this->spiDrOnChan0->spidrReceiveCommand(*commSPIParams.guiData);
        this->waitForNMSecs(25);
    }
    temp.append(this->spiDrOnChan0->getResponse());
    this->commSPIParams.guiData->clear();
    this->commSPIParams.guiData->append("g");  // this one sets the stream of responses again to the state of ST4
    this->spiDrOnChan0->spidrReceiveCommand(*commSPIParams.guiData);
    this->waitForNMSecs(25);
    lastC = this->spiDrOnChan0->getResponse(); // the arduino only provides the byte in the next call, therefore the series ...
    if (lastC=='-') {
        lastC = '0';
    }
    temp.append(lastC);
    lTmp=temp.toDouble();
    if (lTmp < 80) {
        if (lTmp > -40) {
            this->temperature = lTmp;
            ui->lcdTemp->display(this->temperature);
        } else {
            this->temperature = 0;
            ui->lcdTemp->display('-');
        }
    } // in case of a readout error, the result is most likely 3 digits; if the '+' - input of the
      // temperature-input is connected to ground, the result is -50 ... which is translated to '-"
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
//---------------------------------------------------------------------
// code for controlling the secondary focus-motorboard via SPI

//---------------------------------------------------------------------
// a few slots for storing parameters
void MainWindow::storeAuxBoardParams(void) {
    QString *leTxt;
    long denom=16;
    short whichIsGuiderDrive=0;

    leTxt = new QString(ui->leNameAux1->text());
    g_AllData->setAuxName(0,*leTxt);
    leTxt->clear();
    leTxt->append(ui->leNameAux2->text());
    g_AllData->setAuxName(1,*leTxt);
    g_AllData->setStepsToBeDone(0,ui->sbStepsAux1->value());
    g_AllData->setStepsToBeDone(1,ui->sbStepsAux2->value());
    g_AllData->setAuxAcc((ui->sbAuxAcc->value()));
    g_AllData->setAuxSpeed(ui->sbAuxSpeed->value());
    delete leTxt;
    if (ui->rbAuxMs1->isChecked() == true) {
        denom = 1;
    }
    if (ui->rbAuxMs2->isChecked() == true) {
        denom = 2;
    }
    if (ui->rbAuxMs4->isChecked() == true) {
        denom = 4;
    }
    if (ui->rbAuxMs8->isChecked() == true) {
        denom = 8;
    }
    if (ui->rbAuxMs16->isChecked() == true) {
        denom = 16;
    }
    if (ui->rbAuxMs32->isChecked() == true) {
        denom = 32;
    }
    g_AllData->setAuxMSteps(denom);
    if (ui->rbNoFinderFocuser->isChecked() == true) {
        whichIsGuiderDrive = 0;
    }
    if (ui->rbNo1FinderFocuser->isChecked() == true) {
        whichIsGuiderDrive = 1;
    }
    if (ui->rbNo2FinderFocuser->isChecked() == true) {
        whichIsGuiderDrive = 2;
    }
    g_AllData->setGuiderFocusDrive(whichIsGuiderDrive);
    g_AllData->storeGlobalData();
    this->sendAccToAuxController();
    this->sendSpeedToAuxController();
    this->sendMicrostepsToController();
}

//----------------------------------------------------------------------
// send a frequent request whether drives are up ...
void MainWindow::checkDrivesForActivity(void) {
    this->commSPIParams.guiData->clear();
    this->commSPIParams.guiData->append("d");
    this->spiDrOnChan1->spidrReceiveCommand(*commSPIParams.guiData);
    this->waitForNMSecs(10);
}

//----------------------------------------------------------------------
// just checks whether an arduino is connected
bool MainWindow::checkForController(void) {
    char reply;

    this->commSPIParams.guiData->clear();
    this->commSPIParams.guiData->append("ttt");
    this->spiDrOnChan1->spidrReceiveCommand(*commSPIParams.guiData);
    this->waitForNMSecs(50);
    reply = this->spiDrOnChan1->getResponse();
    if ((reply=='D') || (reply=='A')) {
        ui->cbAuxBoardIsConnected->setChecked(true);
        this->auxBoardIsAvailable = true;
        if (reply == 'A') {
            ui->rbAuxMs32->setEnabled(false);
        }
    }

    return this->auxBoardIsAvailable;
}

//-----------------------------------------------------------------------
// sends the number of steps to the controller
// "driveAddressed" is 0 or 1, , "isInverted" inverts the direction of travel,
// and "whatTravel" is 0,1 or 2 in dependence of the of the fraction of travel one wants to do ...

void MainWindow::sendStepsToAuxController(short driveAddressed, bool isInverted, short whatTravel) {
    short sign = 1;
    float fractionOfTravel = 1;

    if ((driveAddressed == 0) || (driveAddressed == 1)) {
        this->commSPIParams.guiData->clear();
        this->commSPIParams.guiData->append("s");
        this->commSPIParams.guiData->append(QString::number(driveAddressed));
        if (isInverted == true) {
            sign = -1;
        }
        switch (whatTravel) {
            case 0: fractionOfTravel = 1; break;
            case 1: fractionOfTravel = 1/5.0; break;
            case 2: fractionOfTravel = 1/20.0; break;
            default: fractionOfTravel = 1; break;
        }

        this->commSPIParams.guiData->append(QString::number(sign*round(g_AllData->getStepsToBeDone(driveAddressed)*fractionOfTravel)));
        this->spiDrOnChan1->spidrReceiveCommand(*commSPIParams.guiData);
        this->waitForNMSecs(150);
    }
}

//-----------------------------------------------------------------------
// send acceleration to controller

void MainWindow::sendAccToAuxController(void) {
    this->commSPIParams.guiData->clear();
    this->commSPIParams.guiData->append("a");
    this->commSPIParams.guiData->append(QString::number(0));
    this->commSPIParams.guiData->append(QString::number(g_AllData->getAuxAcc()));
    this->spiDrOnChan1->spidrReceiveCommand(*commSPIParams.guiData);
    this->waitForNMSecs(150);
    this->commSPIParams.guiData->clear();
    this->commSPIParams.guiData->append("a");
    this->commSPIParams.guiData->append(QString::number(1));
    this->commSPIParams.guiData->append(QString::number(g_AllData->getAuxAcc()));
    this->spiDrOnChan1->spidrReceiveCommand(*commSPIParams.guiData);
    this->waitForNMSecs(150);
}

//-------------------------------------------------------------------------
// send speed to controller

void MainWindow::sendSpeedToAuxController(void) {
    this->commSPIParams.guiData->clear();
    this->commSPIParams.guiData->append("v");
    this->commSPIParams.guiData->append(QString::number(0));
    this->commSPIParams.guiData->append(QString::number(g_AllData->getAuxSpeed()));
    this->spiDrOnChan1->spidrReceiveCommand(*commSPIParams.guiData);
    this->waitForNMSecs(150);
    this->commSPIParams.guiData->clear();
    this->commSPIParams.guiData->append("v");
    this->commSPIParams.guiData->append(QString::number(1));
    this->commSPIParams.guiData->append(QString::number(g_AllData->getAuxSpeed()));
    this->spiDrOnChan1->spidrReceiveCommand(*commSPIParams.guiData);
    this->waitForNMSecs(150);
}

//--------------------------------------------------------------------------
// send microsteps to controller

void MainWindow::sendMicrostepsToController(void) {
    QString *val;

    val = new QString("m ");
    if (ui->rbAuxMs1->isChecked()) {
        val->append("001");
    }
    if (ui->rbAuxMs2->isChecked()) {
        val->append("002");
    }
    if (ui->rbAuxMs4->isChecked()) {
        val->append("004");
    }
    if (ui->rbAuxMs8->isChecked()) {
        val->append("008");
    }
    if (ui->rbAuxMs16->isChecked()) {
        val->append("016");
    }
    if (ui->rbAuxMs32->isChecked()) {
        val->append("032");
    }
    this->commSPIParams.guiData->clear();
    this->commSPIParams.guiData->append(val);
    delete val;
    this->spiDrOnChan1->spidrReceiveCommand(*commSPIParams.guiData);
    this->waitForNMSecs(150);
}

//---------------------------------------------
// enable or disable drives

void MainWindow::enableAuxDrives(short driveAddressed, bool isEnabled) {
    if ((driveAddressed == 0) || (driveAddressed == 1)) {
        this->commSPIParams.guiData->clear();
        this->commSPIParams.guiData->append("e");
        this->commSPIParams.guiData->append(QString::number(driveAddressed));
        if (isEnabled == true) {
            this->commSPIParams.guiData->append('1');
        } else {
            this->commSPIParams.guiData->append('0');
        }
        this->spiDrOnChan1->spidrReceiveCommand(*commSPIParams.guiData);
        this->waitForNMSecs(150);
    }
}

//----------------------------------------------
// start an aux drive

void MainWindow::moveAuxDrive(short driveAddressed) {
    if ((driveAddressed == 0) || (driveAddressed == 1)) {
        this->commSPIParams.guiData->clear();
        this->commSPIParams.guiData->append("o");
        this->commSPIParams.guiData->append(QString::number(driveAddressed));
        this->spiDrOnChan1->spidrReceiveCommand(*commSPIParams.guiData);
        this->waitForNMSecs(150);
    }
}

//----------------------------------------------
// stop an aux drive

void MainWindow::stopAuxDrive(short driveAddressed) {
    if ((driveAddressed == 0) || (driveAddressed == 1)) {
        // something needs to be done for inversion!!!
        this->commSPIParams.guiData->clear();
        this->commSPIParams.guiData->append("x");
        this->commSPIParams.guiData->append(QString::number(driveAddressed));
        this->spiDrOnChan1->spidrReceiveCommand(*commSPIParams.guiData);
        this->waitForNMSecs(150);
        this->enableAuxDrives(driveAddressed, false);
    }
}

//-----------------------------------------------
// slot for an emergency stop
void MainWindow::emergencyStopAuxDrives(void) {
    stopAuxDrive(0);
    stopAuxDrive(1);
}

//-----------------------------------------------
// slots for motion control. dist is 0,1, or 2 for full
// travel, small travel or tiny travel called from the
// configuration menu
void MainWindow::moveAuxPBSlot(short whichDrive, bool isInverted, short dist) {

    this->setAuxDriveControls(false);
    this->auxDriveIsStartingUp=true;  // a flag that suppresses GUI updates in the respective method
    QCoreApplication::processEvents(QEventLoop::AllEvents,100);
    this->enableAuxDrives(whichDrive,true);
    this->storeAuxBoardParams();
    this->sendStepsToAuxController(whichDrive, isInverted, dist);
    this->moveAuxDrive(whichDrive);
    this->auxDriveIsStartingUp=false;
}

//-----------------------------------------------
// same as above, but without saving of parameters - to be called from the guider menu.
void MainWindow::moveGuiderAuxPBSlot(short whichDrive, bool isInverted, short dist) {

    this->setAuxDriveControls(false);
    this->auxDriveIsStartingUp=true;  // a flag that suppresses GUI updates in the respective method
    QCoreApplication::processEvents(QEventLoop::AllEvents,100);
    this->enableAuxDrives(whichDrive,true);
    this->sendStepsToAuxController(whichDrive, isInverted, dist);
    this->moveAuxDrive(whichDrive);
    this->auxDriveIsStartingUp=false;
}
//-----------------------------------------------
void MainWindow::mvAux1FwdFull(void) {
    moveAuxPBSlot(0,false,0);
}

//----------------------------------------------- // same as above, but without parameter saving
void MainWindow::mvAux1FwdFullHB(void) {
    moveGuiderAuxPBSlot(0,false,0);
}
//-----------------------------------------------
void MainWindow::mvAux1BwdFull(void) {
    moveAuxPBSlot(0,true,0);
}

//-----------------------------------------------
void MainWindow::mvAux1BwdFullHB(void) {
    moveGuiderAuxPBSlot(0,true,0);
}

//-----------------------------------------------
void MainWindow::mvAux2FwdFull(void) {
    moveAuxPBSlot(1,false,0);
}

//-----------------------------------------------
void MainWindow::mvAux2FwdFullHB(void) {
    moveGuiderAuxPBSlot(1,false,0);
}

//-----------------------------------------------
void MainWindow::mvAux2BwdFull(void) {
    moveAuxPBSlot(1,true,0);
}

//-----------------------------------------------
void MainWindow::mvAux2BwdFullHB(void) {
    moveGuiderAuxPBSlot(1,true,0);
}
//-----------------------------------------------
void MainWindow::mvAux1FwdSmall(void) {
    moveAuxPBSlot(0,false,1);
}

//-----------------------------------------------
void MainWindow::mvAux1FwdSmallHB(void) {
    moveGuiderAuxPBSlot(0,false,1);
}

//-----------------------------------------------
void MainWindow::mvAux1BwdSmall(void) {
    moveAuxPBSlot(0,true,1);
}

//-----------------------------------------------
void MainWindow::mvAux1BwdSmallHB(void) {
    moveGuiderAuxPBSlot(0,true,1);
}

//-----------------------------------------------
void MainWindow::mvAux2FwdSmall(void) {
    moveAuxPBSlot(1,false,1);
}

//-----------------------------------------------
void MainWindow::mvAux2FwdSmallHB(void) {
    moveGuiderAuxPBSlot(1,false,1);
}
//-----------------------------------------------
void MainWindow::mvAux2BwdSmall(void) {
    moveAuxPBSlot(1,true,1);
}

//-----------------------------------------------
void MainWindow::mvAux2BwdSmallHB(void) {
    moveGuiderAuxPBSlot(1,true,1);
}

//-----------------------------------------------
void MainWindow::mvAux1FwdTiny(void) {
    moveAuxPBSlot(0,false,2);
}

//-----------------------------------------------
void MainWindow::mvAux1FwdTinyHB(void) {
    moveGuiderAuxPBSlot(0,false,2);
}

//-----------------------------------------------
void MainWindow::mvAux1BwdTiny(void) {
    moveAuxPBSlot(0,true,2);
}

//-----------------------------------------------
void MainWindow::mvAux1BwdTinyHB(void) {
    moveGuiderAuxPBSlot(0,true,2);
}

//-----------------------------------------------
void MainWindow::mvAux2FwdTiny(void) {
    moveAuxPBSlot(1,false,2);
}

//-----------------------------------------------
void MainWindow::mvAux2FwdTinyHB(void) {
    moveGuiderAuxPBSlot(1,false,2);
}
//-----------------------------------------------
void MainWindow::mvAux2BwdTiny(void) {
    moveAuxPBSlot(1,true,2);
}

//-----------------------------------------------
void MainWindow::mvAux2BwdTinyHB(void) {
    moveGuiderAuxPBSlot(1,true,2);
}

//-----------------------------------------------
void MainWindow::mvGuideAuxFwdFull(void) {
    if (this->auxBoardIsAvailable == true) {
        if (ui->rbNo1FinderFocuser->isChecked()) {
            moveGuiderAuxPBSlot(0,false,0);
        }
        if (ui->rbNo2FinderFocuser->isChecked()) {
            moveGuiderAuxPBSlot(1,false,0);
        }
    }
}

//-----------------------------------------------
// a slot that updates information on the aux drives
void MainWindow::mvGuideAuxBwdFull(void) {
    if (this->auxBoardIsAvailable == true) {
        if (ui->rbNo1FinderFocuser->isChecked()) {
            moveGuiderAuxPBSlot(0,true,0);
        }
        if (ui->rbNo2FinderFocuser->isChecked()) {
            moveGuiderAuxPBSlot(1,true,0);
        }
    }
}

//-----------------------------------------------
void MainWindow::mvGuideAuxFwdSmall(void) {
    if (this->auxBoardIsAvailable == true) {
        if (ui->rbNo1FinderFocuser->isChecked()) {
            moveGuiderAuxPBSlot(0,false,1);
        }
        if (ui->rbNo2FinderFocuser->isChecked()) {
            moveGuiderAuxPBSlot(1,false,1);
        }
    }
}

//-----------------------------------------------
void MainWindow::mvGuideAuxBwdSmall(void) {
    if (this->auxBoardIsAvailable == true) {
        if (ui->rbNo1FinderFocuser->isChecked()) {
            moveGuiderAuxPBSlot(0,true,1);
        }
        if (ui->rbNo2FinderFocuser->isChecked()) {
            moveGuiderAuxPBSlot(1,true,1);
        }
    }
}

//-----------------------------------------------
void MainWindow::mvGuideAuxFwdTiny(void) {
    if (this->auxBoardIsAvailable == true) {
        if (ui->rbNo1FinderFocuser->isChecked()) {
            moveGuiderAuxPBSlot(0,false,2);
        }
        if (ui->rbNo2FinderFocuser->isChecked()) {
            moveGuiderAuxPBSlot(1,false,2);
        }
    }
}

//-----------------------------------------------
void MainWindow::mvGuideAuxBwdTiny(void) {
    if (this->auxBoardIsAvailable == true) {
        if (ui->rbNo1FinderFocuser->isChecked()) {
            moveGuiderAuxPBSlot(0,true,2);
        }
        if (ui->rbNo2FinderFocuser->isChecked()) {
            moveGuiderAuxPBSlot(1,true,2);
        }
    }
}

//------------------------------------------------
void MainWindow::updateAuxDriveStatus(void) {
    char focusMotorReply1, focusMotorReply2;

    if (this->auxBoardIsAvailable == true) { // if the motorboard is connected - display the motorstatus
        this->spiDrOnChan1->spidrReceiveCommand("ttt");
        focusMotorReply1 = this->spiDrOnChan1->getResponse();
        this->spiDrOnChan1->spidrReceiveCommand("ttt");
        focusMotorReply2 = this->spiDrOnChan1->getResponse(); // SPI is flaky - in order to make sure that the response is correct, it is called twice ...
        if (focusMotorReply1 == focusMotorReply2) {
            switch (focusMotorReply2) {
                case 'D':
                case 'A':
                    ui->cbAuxDr1Active->setChecked(false);
                    ui->cbAuxDr2Active->setChecked(false);
                    setAuxDriveControls(true);
                    break;
                case '0':
                    ui->cbAuxDr1Active->setChecked(true);
                    ui->cbAuxDr2Active->setChecked(false);
                    setAuxDriveControls(false);
                    break;
                case '1':
                    ui->cbAuxDr1Active->setChecked(false);
                    ui->cbAuxDr2Active->setChecked(true);
                    setAuxDriveControls(false);
                    break;
                case 'B':
                    ui->cbAuxDr1Active->setChecked(true);
                    ui->cbAuxDr2Active->setChecked(true);
                    setAuxDriveControls(false);
                    break;
            }
        }
    }
}

//-------------------------------------------------------------------------
// a slot that stores the stae of the checkbox on using LX200 by default upon startup

void MainWindow::handleSerialLXCB(void) {
    if (ui->cbSerialLX200Default->isChecked() == true) {
        g_AllData->setLX200SerialFlag(true);
    } else {
        g_AllData->setLX200SerialFlag(false);
    }
    g_AllData->storeGlobalData();
}

//-------------------------------------------------------------------------
// three slots for handling the meridian flip ...

void MainWindow::mountIsGerman(void) {
    if (ui->cbIsGEM->isChecked() == true) {
        ui->cbMountIsEast->setEnabled(true);
        g_AllData->setMFlipParams(0,true); //0 is "isGEM", 1 is "isEast"
        g_AllData->setMFlipParams(1,true);
        ui->cbMountIsEast->setChecked(true);
    } else {
        ui->cbMountIsEast->setEnabled(false);
        g_AllData->setMFlipParams(0,false); //0 is "isGEM", 1 is "isEast"
    }
    g_AllData->storeGlobalData(); // save the GEM state to the preferences
}

//-------------------------------------------------------------------------
void MainWindow::mountIsEast(void) {

    if (ui->cbIsGEM->isChecked() == true) {
        if (ui->cbMountIsEast->isChecked() == true) {
            g_AllData->setMFlipParams(1,true);
            g_AllData->setDeclinationSign(1); // sets the declination sign
        } else {
            g_AllData->setMFlipParams(1,false);
            g_AllData->setDeclinationSign(-1); // sets the declination sign
        }
    }
}

//-------------------------------------------------------------------------
void MainWindow::setDecForNoFlip(void) {
    g_AllData->setMaxDeclForNoFlip(ui->sbNoFlip->value());
    g_AllData->storeGlobalData(); // save the value to the preferences
}

//-------------------------------------------------------------------------
// a slot for storing whether location and time can be set from LX200
void MainWindow::setTimeFromLX200Flag(void) {
    if (ui->cbTimeFromLX200->isChecked() == true) {
        g_AllData->setTimeFromLX200Flag(true);
    } else {
        g_AllData->setTimeFromLX200Flag(false);
    }
    g_AllData->storeGlobalData(); // save the value to the preferences
}

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
// plate solving routines -------------------------------------------------
// this is a dummy by now ...
void MainWindow::psTakeImage(void) {

    ui->pbSolveFieldPS->setEnabled(false);
    ui->pbSyncPS->setEnabled(false);
    ui->cbImageReceived->setChecked(false);
    ui->cbPSImageInTransfer->setChecked(false);
    if (g_AllData->getPathToImages().length() != 0) {
        this->camera_client->takeExposure((ui->sbPSExposureTime->value()), true);
        this->ccdMainCameraIsAcquiring=true;
        ui->sbPSExposureTime->setEnabled(false);
        ui->pbTakeImagePS->setEnabled(false);
        this->elapsedPS->restart();
    } else {
        qDebug() << "no path to images available";
    }
}

//--------------------------------------------------------------------------
// slot for choosing a directory
void MainWindow::psChooseFITSDirectory(void) {
    QFileDialog dialog;
    QString theChosenPath;

    dialog.setFileMode(QFileDialog::DirectoryOnly);
    dialog.setOption(QFileDialog::ShowDirsOnly, true);
    dialog.exec();
    if (dialog.selectedFiles().empty() == false) {
        theChosenPath = (dialog.selectedFiles().at(0));
        const QFileInfo newDir(theChosenPath.toLatin1()) ;
        if (newDir.exists() && newDir.isDir() && newDir.isReadable() && newDir.isWritable()) {
            theChosenPath.append("/");
            g_AllData->setPathToImages(theChosenPath);
            ui->lePathToFitsFile->setText(g_AllData->getPathToImages());
        }
    }
}

//---------------------------------------------------------------------------
// slot that triggers astrometry.net
void MainWindow::psStartSolving(void) {
    QString* solveCommand;

    double fieldSize, slow, maxSize;
    short dsFact = 1;

    ui->cbFieldSolved->setChecked(false);
    ui->pbKillAMetry->setEnabled(true);
    ui->cbPSInProgress->setChecked(true);
    g_AllData->setBooleanPSParams(0,true); // platesolving in progress
    g_AllData->setBooleanPSParams(1,false); // no success so far
    g_AllData->setBooleanPSParams(2, false); // no error so far
    if (g_AllData->getCameraPixelSize(0,true) >= g_AllData->getCameraPixelSize(1,true)) {
        maxSize = (g_AllData->getCameraChipPixels(0,true));
    } else {
        maxSize = (g_AllData->getCameraChipPixels(1,true));
    }
    if (maxSize > 2000) {
        dsFact = 2;
    }
    if (maxSize > 4000) {
        dsFact = 4;
    }
    this->waitForNMSecs(250);
    fieldSize = 3*this->psComputeFOVForMainCCD();
    slow = fieldSize/5.0;
    solveCommand = new QString("solve-field --ra ");
    solveCommand->append(QString::number((double)g_AllData->getActualScopePosition(2),'g',6));
    solveCommand->append(" --dec ");
    solveCommand->append(QString::number((double)g_AllData->getActualScopePosition(1),'g',6));
    solveCommand->append(" --radius ");
    solveCommand->append(QString::number((double)(ui->sbPSSearchRad->value())));
    solveCommand->append(" --scale-units degwidth --overwrite --match none --rdls none --wcs none --no-plots ");
    solveCommand->append(" --scale-low ");
    solveCommand->append(QString::number(slow));
    solveCommand->append(" --scale-high ");
    solveCommand->append(QString::number(fieldSize));
    solveCommand->append(" --downsample ");
    solveCommand->append(QString::number(dsFact));
    solveCommand->append(" ");
    solveCommand->append(g_AllData->getPathToImageToBeSolved().toLatin1());
    qDebug() << "Process command: " << solveCommand->toLatin1();
    this->astroMetryProcess->start(solveCommand->toLatin1());
    if (!this->astroMetryProcess->waitForStarted()) {
        g_AllData->setBooleanPSParams(0,false);
        g_AllData->setBooleanPSParams(2, true);
        qDebug() << "Process did not start ...";
    }
    delete solveCommand;
    ui->Control->setCursor(Qt::BusyCursor);
    connect(this->astroMetryProcess, SIGNAL(readyReadStandardOutput()), this, SLOT(psDisplayAstrometryNetOutput()));
}

//-----------------------------------------------------------------------------
// a slot for setting the search radius
void MainWindow::psSetSearchRadiusForPS(void) {
    double val;

    val = ui->sbPSSearchRad->value();
    g_AllData->setPSSearchRad(val);
}

//-----------------------------------------------------------------------------
// a slot that handles the end of the astrometry.net process
void MainWindow::psHandleEndOfAstronomyNetProcess(int code , QProcess::ExitStatus status) {
    QMessageBox amnetCrashMsg;
    QStringList nameFilter("*.solved");
    QDir *psDirectory;
    QStringList solvedFiles;

    qDebug() << "Astrometry.net code: " << code;
    this->waitForNMSecs(1000); // just take a little breath
    psDirectory = new QDir(g_AllData->getPathToImages().toLatin1());
    solvedFiles = psDirectory->entryList(nameFilter);
    delete psDirectory; // searching for a .solved file in the directory where astrometry.net operates

    if (status == QProcess::NormalExit) {
        if (solvedFiles.isEmpty() == false) {
            qDebug() << ".solved file found";
            ui->cbFieldSolved->setChecked(true);
            this->psreadCoordinatesFromFITS();
        }
    } else {
        amnetCrashMsg.setWindowTitle("Astrometry net error");
        amnetCrashMsg.setText("Astrometry.net engine crashed or terminated...");
        amnetCrashMsg.exec();
    }
    ui->cbPSInProgress->setChecked(false);
    ui->pbKillAMetry->setEnabled(false);
    ui->Control->setCursor(Qt::ArrowCursor);
}

//--------------------------------------------------------------------------------
// compute the field of view and derive the oversampling factor and the index range from this
double MainWindow::psComputeFOVForMainCCD(void) {
    double maxSize, fov;

    if (g_AllData->getCameraPixelSize(0,true) >= g_AllData->getCameraPixelSize(1,true)) {
        maxSize = (g_AllData->getCameraChipPixels(0,true) * g_AllData->getCameraPixelSize(0,true))/1000.0;
    } else {
        maxSize = (g_AllData->getCameraChipPixels(1,true) * g_AllData->getCameraPixelSize(1,true))/1000.0; // find the maximum chip dimension in mm
    }
    fov = 34.60*maxSize/(g_AllData->getMainScopeFocalLength());
    qDebug() << "Field of View [°] is: " << fov;
    qDebug() << "Max. Chip Size: " << maxSize;
    return fov;
}

//--------------------------------------------------------------------------------------
// a slot that kills astrometry.net
void MainWindow::psKillAstrometryNet(void) {
    this->astroMetryProcess->terminate();
    ui->cbPSInProgress->setEnabled(false);
    ui->pbKillAMetry->setEnabled(false);
}

//--------------------------------------------------------------------------------------
// a slot that handles astrometry.net output
void MainWindow::psDisplayAstrometryNetOutput(void) {
    QProcess *p = dynamic_cast<QProcess *>( sender() );
    QString *line;

    if (p) {
      line = new QString(p->readAllStandardOutput());
      ui->tbAstroMetry->append(line->toLatin1());
      delete line;
    }
}

//--------------------------------------------------------------------------------------
// a slot that handles wcsinfo output
void MainWindow::psReadWCSInfoOutput(void) {
    QProcess *p = dynamic_cast<QProcess *>( sender() );
    QString *line;

    if (p) {
      line = new QString(p->readAllStandardOutput());
      this->wcsInfoOutput->append(line);
      delete line;
    }
}

//---------------------------------------------------------------------------------------
// read the coordinates from the FITS file after solving; read EQUINOX, CRVAL1 and CRVAL2 from header
void MainWindow::psreadCoordinatesFromFITS(void) {
    QString *newFileName, *datastring, *wcsProcess, *raString, *deString;
    float equinox, solvedRA = 0, solvedDec = 0;
    bool solvedCenterCoordsFound = false;
    double meeusM, meeusN, deltaRA, deltaDecl,raRadians, declRadians,corrRA, corrDecl;
    QProcess *readWCSInfo;
    QStringList wcsResults;
    int idx;

    qDebug() << "Reading Platesolver coordinates...";
    newFileName = new QString(g_AllData->getPathToImageToBeSolved().toLatin1());
    wcsProcess = new QString("wcsinfo ");
    datastring = new QString();
    if (newFileName->isEmpty() == false) {
        newFileName->chop(3);
        newFileName->append("new");

        wcsProcess->append(newFileName);
        readWCSInfo = new QProcess();
        readWCSInfo->start(wcsProcess->toLatin1());
        if (!readWCSInfo->waitForStarted()) {
            qDebug() << "Cannot read wcs data...";
        } else {
            connect(readWCSInfo, SIGNAL(readyReadStandardOutput()), this, SLOT(psReadWCSInfoOutput()));
            do {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 1000);
            } while (!readWCSInfo->waitForFinished() );
            delete readWCSInfo;
            wcsResults = this->wcsInfoOutput->split('\n');
            for (idx = 0; idx < wcsResults.size(); idx++) {
                datastring->append(wcsResults.at(idx));
                if (datastring->startsWith("ra_center ") == true) {
                    datastring->remove("ra_center ");
                    solvedRA = datastring->toFloat();
                }
                if (datastring->startsWith("dec_center ") == true) {
                    datastring->remove("dec_center ");
                    solvedDec = datastring->toFloat();
                    solvedCenterCoordsFound = true;
                }
                datastring->clear();
            }
            if (solvedCenterCoordsFound) {
                equinox = this->UTDate->year();
                meeusM=(3.07234+0.00186*((equinox-1900)/100.0))*0.00416667; // factor m, J. Meeus, 3. ed, p.63, given in degrees
                meeusN=(20.0468-0.0085*((equinox-1900)/100.0))/(3600.0);  // factor n, in degrees
                raRadians=solvedRA/180.0*3.141592653589793;
                declRadians=solvedDec/180.0*3.141592653589793;
                deltaRA = meeusM+meeusN*sin(raRadians)*tan(declRadians);
                deltaDecl = meeusN*cos(raRadians);
                corrRA= solvedRA+deltaRA*((double)(this->UTDate->year()-2000));
                if (corrRA > 360) {
                    corrRA-=360;
                } // that one is clear,right - avoid more than 24h RA ...
                corrDecl=solvedDec+deltaDecl*((double)(this->UTDate->year()-2000));
                if (corrDecl > 90) {
                    corrDecl = 90; // don't know what else to do here
                }
                if (corrDecl <-90) {
                    corrDecl = -90;
                }
                raString = new QString(*this->generateCoordinateString(corrRA, true));
                deString=new QString(*this->generateCoordinateString(corrDecl, false));
                ui->lePSRASolved->setText(*raString);
                ui->lePSDeclSolved->setText(*deString);
                delete raString;
                delete deString;
                this->psRA = corrRA;
                this->psDecl = corrDecl;
            }
        }
    }
    delete wcsProcess;
    delete datastring;
    delete newFileName;
    ui->pbSyncPS->setEnabled(true);
    g_AllData->setBooleanPSParams(false, 0);
    g_AllData->setBooleanPSParams(true, 1);
    g_AllData->setBooleanPSParams(false, 2);
}

//-----------------------------------------------------------------------------------------------
// a slot that syncs to the found coordinates
void MainWindow::syncPSCoordinates(void) {
    if (g_AllData->getBooleanPSParams(1) == true) {
        this->ra = this->psRA;
        this->decl = this->psDecl;
        this->syncMount();
        ui->pbSyncPS->setEnabled(false);
    }
}

//------------------------------------------------------------------
// this slot receives a pixmap from the camera client from the MainCamera and uses it for
// display in the Plate Solving tab
void MainWindow::displayMainCamImage(QPixmap *camPixmap) {

    if (g_AllData->getINDIState(true) == true) { // ... if the main camera is connected to the INDI server ...
        delete mainCamImg;
        this->mainCamImg = new QPixmap(*camPixmap);
        ui->lDisplayPSCam->setPixmap(*mainCamImg); // receive the pixmap from the camera ...
        ui->pbSolveFieldPS->setEnabled(true);
        ui->cbImageReceived->setChecked(true);
        ui->cbPSImageInTransfer->setChecked(false);
        ui->sbPSExposureTime->setEnabled(true);
        this->ccdMainCameraIsAcquiring=false;
        ui->pbTakeImagePS->setEnabled(true);
        this->psImageAcquisionTimeRemaining = 0;
        ui->lcdETAOfPSImage->display(0);
    }
}

//------------------------------------------------------------------

