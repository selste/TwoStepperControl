#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "qstepperphidgetsRA.h"
#include "qstepperphidgetsDecl.h"
#include <qtimer.h>
#include <QtConcurrent/qtconcurrentrun.h>
#include <QDir>
#include <math.h>
#include <unistd.h>
#include "QDisplay2D.h"
#include "tsc_globaldata.h"

TSC_GlobalData *g_AllData; // a global class that holds system specific parameters on drive, current mount position, gears and so on ...

//------------------------------------------------------------------
// constructor of the GUI - takes care of everything....
MainWindow::MainWindow(QWidget *parent):QMainWindow(parent), ui(new Ui::MainWindow)
{
    int serNo; // serial number of the phidgets boards
    double val,draccRA, draccDecl, drcurrRA, drcurrDecl; // local values on drive acceleration and so on...
    QStepperPhidgetsRA *dummyDrive; // a helper to determine the right order of drives
    QDir *catalogDir; // the directory holding the local .tsc catalogs
    QFileInfoList catFiles; // a list of available .tsc files
    QFileInfo catFileInfo; // a helper on the file list of catalogs
    QStringList filter; // needed for isolating the .tsc files
    QString *catfName; // name of a .tsc catalog

    ui->setupUi(this); // making the widget
    g_AllData =new TSC_GlobalData(); // instantiate the global class with parameters
    QTimer *timer = new QTimer(this); // start the event timer ... this is NOT the microtimer for the mount
    timer->start(100); // check all 100 ms for events
    elapsedGoToTime = new QElapsedTimer(); // timer for roughly measuring time taked during GoTo

    draccRA= g_AllData->getDriveParams(0,1);
    draccDecl= g_AllData->getDriveParams(1,1);
    drcurrRA= g_AllData->getDriveParams(0,2);
    drcurrDecl= g_AllData->getDriveParams(1,2); // retrieving acceleration and maximum current for the phidget boards

    // start searching for the right boards for the drives ...
    dummyDrive = new QStepperPhidgetsRA(draccRA,drcurrRA); // call the first phidget interface to the board of the stepper
    serNo = dummyDrive->retrievePhidgetStepperData(1);
    delete dummyDrive;
    if ((g_AllData->getDriveID(0) != serNo) && (g_AllData->getDriveID(1) != serNo)) { //no driver boards are assigned to drives
        StepperDriveRA = new QStepperPhidgetsRA(draccRA,drcurrRA); // call the phidget interface to the board of the stepper
        serNo = StepperDriveRA->retrievePhidgetStepperData(1);     // get the serial number of the phidget board
        g_AllData->setDriveData(0,serNo); // remember the ID of the RA-drive in the global class

        StepperDriveDecl = new QStepperPhidgetsDecl(draccDecl,drcurrDecl); // call the phidget interface to the board of the stepper
        serNo = StepperDriveDecl->retrievePhidgetStepperData(1); // get the serial number of the phidget board
        g_AllData->setDriveData(1,serNo); // remember the ID of the Decl-drive in the global class

        ui->lcdRAID->display(QString::number(g_AllData->getDriveID(0)));
        ui->lcdDeclID->display(QString::number(g_AllData->getDriveID(1)));  // display the IDs in the LCD-display on the "Drive" tab
    } else { // IDs are written in the "TSC_Preferences.tsc" file
        dummyDrive = new QStepperPhidgetsRA(draccRA,drcurrRA); // call the first phidget interface to the board of the stepper
        serNo = dummyDrive->retrievePhidgetStepperData(1);
        if (serNo != g_AllData->getDriveID(0)) { // dummy drive is NOT the designatedRA Drive
            StepperDriveRA = new QStepperPhidgetsRA(draccRA,drcurrRA); // call the phidget interface to the board of the stepper
            serNo =StepperDriveRA->retrievePhidgetStepperData(1);
            g_AllData->setDriveData(0,serNo);
            ui->lcdRAID->display(QString::number(g_AllData->getDriveID(0)));
            delete dummyDrive; // set the other board to RA
            StepperDriveDecl = new QStepperPhidgetsDecl(draccDecl,drcurrDecl); // call the phidget interface to the board of the stepper
            serNo = StepperDriveDecl->retrievePhidgetStepperData(1);
            g_AllData->setDriveData(1,serNo);
            ui->lcdDeclID->display(QString::number(g_AllData->getDriveID(1)));
        } else {
            StepperDriveDecl = new QStepperPhidgetsDecl(draccDecl,drcurrDecl); // call the phidget interface to the board of the stepper
            serNo = StepperDriveDecl->retrievePhidgetStepperData(1);
            g_AllData->setDriveData(1,serNo);
            ui->lcdDeclID->display(QString::number(g_AllData->getDriveID(1)));
            delete dummyDrive; // set the other board to Decl
            StepperDriveRA = new QStepperPhidgetsRA(draccRA,drcurrRA); // call the phidget interface to the board of the stepper
            serNo =StepperDriveRA->retrievePhidgetStepperData(1);
            g_AllData->setDriveData(0,serNo);
            ui->lcdRAID->display(QString::number(g_AllData->getDriveID(0)));
        }
    }

        // set a bunch of flags and factors
    this->mountMotion.RATrackingIsOn=false;   // sidereal tracking is on if true
    this->mountMotion.RADriveIsMoving =false; // RA drive is moving faster than sideral tracking if true
    this->mountMotion.DeclDriveIsMoving = false; // Decl drive is moving if true
    this->mountMotion.GoToIsActiveInRA = false; // system is in a slew state, RA is moving. most system functionality is disabled
    this->mountMotion.GoToIsActiveInDecl = false; // system is in a slew state, Decl is moving. most system functionality is disabled
    this->mountMotion.emergencyStopTriggered = false; // system can be halted by brute force. true if this was triggered
    this->guidingIsActive=false; // if, autoguiding is on and most system functionality is disabled.
    this->lx200IsOn = false; // true if a serial connection was opened vai RS232
    this->ccdCameraIsAcquiring=false; // true if images are coming in from INDI-server
    this->MountWasSynced = false; // true if mount was synced, either trough internal catalogs or by LX200
    this->mountMotion.DeclDriveDirection = 1; // 1 for forward, -1 for backward
    this->mountMotion.RADriveDirection = 1; // 1 for forward, -1 for backward
    this->mountMotion.RASpeedFactor=1;
    this->mountMotion.DeclSpeedFactor=1; // speeds are multiples of sidereal compensation
    ui->rbCorrSpeed->setChecked(true); // activate radiobutton for correction speed ... this is sidereal speed

        // now setting all the parameters in the "Drive"-tab. settings are from pref-file, except for the stepper speed, which is
        // calculated from  gear parameters
    g_AllData->setDriveParams(0,0,this->StepperDriveRA->getKinetics(3)); // velocity limit - this is set to sidereal speed for right ascension in the constructor of the stepper class ...
    g_AllData->setDriveParams(1,0,this->StepperDriveDecl->getKinetics(3)); // velocity limit - this is set to sidereal speed for declination in the constructor of the stepper class ...
    this->StepperDriveRA->setStepperParams((g_AllData->getDriveParams(0,1)),1); // acceleration in RA
    this->StepperDriveRA->setStepperParams((g_AllData->getDriveParams(0,2)),3); // motor current in RA
    this->StepperDriveDecl->setStepperParams((g_AllData->getDriveParams(1,1)),1); // acceleration in Decl
    this->StepperDriveDecl->setStepperParams((g_AllData->getDriveParams(1,2)),3); // motor current in Decl
    textEntry = new QString();
    val=(this->StepperDriveRA->getKinetics(3));
    ui->leVMaxRA->setText(textEntry->number(val,'f',2));
    textEntry->clear();
    ui->leAMaxRA->setText(textEntry->number(this->StepperDriveRA->getKinetics(2)));
    textEntry->clear();
    ui->leCurrMaxRA->setText(textEntry->number(this->StepperDriveRA->getKinetics(1)));
    textEntry->clear();
    val=(this->StepperDriveDecl->getKinetics(3));
    ui->leVMaxDecl->setText(textEntry->number(val,'f',2));
    textEntry->clear();
    ui->leAMaxDecl->setText(textEntry->number(this->StepperDriveDecl->getKinetics(2)));
    textEntry->clear();
    ui->leCurrMaxDecl->setText(textEntry->number(this->StepperDriveDecl->getKinetics(1)));
    textEntry->clear();
    ui->leRAPlanetary->setText(textEntry->number(g_AllData->getGearData(0)));
    textEntry->clear();
    ui->leRAGear->setText(textEntry->number(g_AllData->getGearData(1)));
    textEntry->clear();
    ui->leRAWorm->setText(textEntry->number(g_AllData->getGearData(2)));
    textEntry->clear();
    ui->leRAStepsize->setText(textEntry->number(g_AllData->getGearData(3)));
    textEntry->clear();
    ui->leDeclPlanetary->setText(textEntry->number(g_AllData->getGearData(4)));
    textEntry->clear();
    ui->leDeclGear->setText(textEntry->number(g_AllData->getGearData(5)));
    textEntry->clear();
    ui->leDeclWorm->setText(textEntry->number(g_AllData->getGearData(6)));
    textEntry->clear();
    ui->leDeclStepSize->setText(textEntry->number(g_AllData->getGearData(7)));
    textEntry->clear();
    ui->leMicrosteps->setText(textEntry->number(g_AllData->getGearData(8)));
    textEntry->clear();
        // now setting all the parameters in the "Cam"-tab
    ui->lePixelSizeX->setText(textEntry->number(g_AllData->getCameraPixelSize(0)));
    textEntry->clear();
    ui->lePixelSizeY->setText(textEntry->number(g_AllData->getCameraPixelSize(1)));
    textEntry->clear();
    ui->leFrameSizeX->setText(textEntry->number(g_AllData->getCameraChipPixels(0)));
    textEntry->clear();
    ui->leFrameSizeY->setText(textEntry->number(g_AllData->getCameraChipPixels(1)));
    textEntry->clear();

        // camera nd guiding class are instantiated
    camera_client = new alccd5_client(); // install a camera client for guiding via INDI
    guiding = new ocv_guiding();
    guideStarPrev = new QPixmap(); // a pixmap for showing the preview window
    guideStarPosition.centrX =0.0;
    guideStarPosition.centrY =0.0;
    this->guidingFOVFactor=1.0; // location of the crosshair and size of the preview window are set
    ui->sbFLGuideScope->setValue(g_AllData->getGuideScopeFocalLength()); // get stored focal length for the guidescope
    this->guiding->setFocalLengthOfGuidescope(g_AllData->getGuideScopeFocalLength());

        // now read all catalog files, ending in "*.tsc"
    catalogDir = new QDir();
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
    this->camView = new QDisplay2D(ui->camTab,425,340); // make the clicakble scene view of 425 x 340 pixels
    this->camImg= new QPixmap(g_AllData->getCameraDisplaySize(0),g_AllData->getCameraDisplaySize(1)); // store the size of the scene view in the global parameter class
    RAdriveDirectionForNorthernHemisphere = 1; //switch this for the southern hemisphere to -1 ... RA is inverted
    g_AllData->storeGlobalData();
    g_AllData->setSyncPosition(0.0, 0.0); // deploy a fake sync to the mount so that the microtimer starts ...

        // instantiate the class for serial communication via LX200
    this->lx200port= new lx200_communication();
    if (this->lx200port->getPortState() == 1) {
        ui->cbRS232Open->setChecked(true);
    }
    this->LXSetNumberFormatToSimple(); // LX200 knows a simple and a complex number format for RA and Decl - set format to simple here ...

        // connecting signals and slots
    connect(timer, SIGNAL(timeout()), this, SLOT(updateReadings())); // this is the event queue
    connect(ui->pbExit,SIGNAL(clicked()), this, SLOT(shutDownProgram())); // this kills the program, including killing the drives
    connect(ui->pbConnectToServer,SIGNAL(clicked()),this, SLOT(setINDISAddrAndPort())); // connects to the INDI server at the given address ...
    connect(ui->pbExpose, SIGNAL(clicked()), this, SLOT(startCCDAcquisition())); // start acquiring images from the guidecam. a signal is emitted if an image arrived.
    connect(ui->pbStopExposure, SIGNAL(clicked()), this, SLOT(stopCCDAcquisition())); // just set the local flag on ccd-acquisition so that no new image is polled in "displayGuideCamImage".
    connect(ui->listWidgetCatalog,SIGNAL(itemClicked(QListWidgetItem*)),this,SLOT(catalogChosen(QListWidgetItem*))); // choose an available .tsc catalog
    connect(ui->listWidgetObject,SIGNAL(itemClicked(QListWidgetItem*)),this,SLOT(catalogObjectChosen(void))); // catalog selection
    connect(this->camView,SIGNAL(currentViewStatusSignal(QPointF)),this->camView,SLOT(currentViewStatusSlot(QPointF))); // position the crosshair in the camera view by mouse...
    connect(this->guiding,SIGNAL(determinedGuideStarCentroid()), this->camView,SLOT(currentViewStatusSlot())); // an overload of the precious slot that allows for positioning the crosshair after a centroid was computed during guiding...
    connect(ui->pbSync, SIGNAL(clicked()), this, SLOT(syncMount())); // reset the current position and global timer, and set the global mount position to the actual coordinates
    connect(ui->pbStoreGears, SIGNAL(clicked()), this, SLOT(storeGearData())); // well - take the data from the dialog and store them in the .tsp file and in g_AllData
    connect(ui->pbStartTracking, SIGNAL(clicked()),this,SLOT(startRATracking())); // start earth motion compensation in RA
    connect(ui->pbStopTracking, SIGNAL(clicked()),this,SLOT(stopRATracking())); // stop earth motion compensation in RA
    connect(ui->pbDeclUp, SIGNAL(clicked()),this,SLOT(declinationMoveHandboxUp())); // manual motion of the handbox - decl up
    connect(ui->pbDeclDown, SIGNAL(clicked()),this,SLOT(declinationMoveHandboxDown())); // manual motion of the handbox - decl down
    connect(ui->pbRAPlus, SIGNAL(clicked()),this,SLOT(RAMoveHandboxFwd())); // manual motion of the handbox - ra towards sunset
    connect(ui->pbRAMinus, SIGNAL(clicked()),this,SLOT(RAMoveHandboxBwd())); // manual motion of the handbox - ra towards dawn
    connect(ui->leAMaxRA, SIGNAL(textChanged(QString)), this, SLOT(setMaxStepperAccRA())); // process input on stepper parameters in gear-tab
    connect(ui->leCurrMaxRA, SIGNAL(textChanged(QString)), this, SLOT(setMaxStepperCurrentRA())); // process input on stepper parameters in gear-tab
    connect(ui->leAMaxDecl, SIGNAL(textChanged(QString)), this, SLOT(setMaxStepperAccDecl())); // process input on stepper parameters in gear-tab
    connect(ui->leCurrMaxDecl, SIGNAL(textChanged(QString)), this, SLOT(setMaxStepperCurrentDecl())); // process input on stepper parameters in gear-tab
    connect(ui->pbStoreDrive, SIGNAL(clicked()), this, SLOT(storeDriveData())); // store data to preferences
    connect(ui->rbCorrSpeed,SIGNAL(released()), this, SLOT(setCorrectionSpeed())); // set speed for slow manual motion
    connect(ui->rbMoveSpeed,SIGNAL(released()), this, SLOT(setMoveSpeed())); // set speed for faster manual motion
    connect(ui->pbGoTo, SIGNAL(clicked()),this, SLOT(startGoToObject())); // start the slew routine
    connect(ui->sbMoveSpeed, SIGNAL(valueChanged(int)),this,SLOT(changeMoveSpeed())); // set factor for faster manual motion
    connect(ui->cbIsOnNorthernHemisphere, SIGNAL(stateChanged(int)), this, SLOT(invertRADirection())); // switch direction of RA motion for the southern hemisphere
    connect(ui->cbStoreGuideCamImgs, SIGNAL(stateChanged(int)), this, SLOT(enableCamImageStorage())); // a checkbox that starts saving all camera images in the camera-class
    connect(ui->pbLX200Active, SIGNAL(clicked()), this, SLOT(switchToLX200())); // open the serial port for LX 200
    connect(ui->pbGetCCDParams, SIGNAL(clicked()), this, SLOT(getCCDParameters())); // read pixel and frame size for the ccd
    connect(ui->pbStoreCCDParams, SIGNAL(clicked()), this, SLOT(storeCCDData())); // store pixel and frame size for the ccd
    connect(ui->pbStartINDIServer, SIGNAL(clicked()), this, SLOT(deployINDICommand())); // call a system command to start an INDI server with given driver parameters
    connect(ui->pbStop1, SIGNAL(clicked()), this, SLOT(emergencyStop())); // kill all motion immediately
    connect(ui->pbStop2, SIGNAL(clicked()), this, SLOT(emergencyStop())); // kill all motion immediately
    connect(ui->pbStop3, SIGNAL(clicked()), this, SLOT(emergencyStop())); // kill all motion immediately
    connect(ui->pbStop4, SIGNAL(clicked()), this, SLOT(emergencyStop())); // kill all motion immediately
    connect(ui->pbPGDecPlus, SIGNAL(clicked()), this, SLOT(declPGPlus())); // pulse guide for a given amount of time defined in a spinbox
    connect(ui->pbPGDecMinus, SIGNAL(clicked()), this, SLOT(declPGMinus())); // pulse guide for a given amount of time defined in a spinbox
    connect(ui->pbPGRAPlus, SIGNAL(clicked()), this, SLOT(raPGFwd())); // pulse guide for a given amount of time defined in a spinbox
    connect(ui->pbPGRAMinus, SIGNAL(clicked()), this, SLOT(raPGBwd())); // pulse guide for a given amount of time defined in a spinbox
    connect(ui->pbClearLXLog, SIGNAL(clicked()), this, SLOT(clearLXLog())); // delete the log of LX200 commands
    connect(ui->pbSelectGuideStar, SIGNAL(clicked()), this, SLOT(selectGuideStar())); // select a guide star defined by crosshair in the QDisplay - widget
    connect(ui->pbGuiding,SIGNAL(clicked()), this, SLOT(doAutoGuiding())); // instantiate all variables for autoguiding and set a flag that takes care of correction in "displayGuideCamImage" and "correctGuideStarPosition"
    connect(ui->hsThreshold,SIGNAL(valueChanged(int)), this, SLOT(changePrevImgProc())); // change threshold for selecting a guidestar
    connect(ui->hsIContrast ,SIGNAL(valueChanged(int)), this, SLOT(changePrevImgProc())); // change contrast for selecting a guidestar
    connect(ui->hsIBrightness ,SIGNAL(valueChanged(int)), this, SLOT(changePrevImgProc())); // change brightness for selecting a guidestar
    connect(ui->cbMedianFilter, SIGNAL(stateChanged(int)), this, SLOT(changePrevImgProc())); // apply a 3x3 median filter to the guidestar - image
    connect(ui->sbFLGuideScope, SIGNAL(valueChanged(int)), this, SLOT(changeGuideScopeFL())); // spinbox for guidescope - focal length
    connect(ui->pbStoreFL, SIGNAL(clicked()), this, SLOT(storeGuideScopeFL())); // store focal length of guidescope to preferences
    connect(ui->rbFOVStd, SIGNAL(released()), this, SLOT(setRegularFOV())); // guidestar window set to 180x180 pixels
    connect(ui->rbFOVHalf, SIGNAL(released()), this, SLOT(setHalfFOV())); // guidestar window set to 90x90 pixels
    connect(ui->rbFOVDbl, SIGNAL(released()), this, SLOT(setDoubleFOV())); // guidestar window set to 360x360 pixels
    connect(this->lx200port,SIGNAL(RS232moveEast()), this, SLOT(LXmoveEast()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232moveWest()), this, SLOT(LXmoveWest()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232moveNorth()), this, SLOT(LXmoveNorth()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232moveSouth()), this, SLOT(LXmoveSouth()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232stopMoveEast()), this, SLOT(LXstopMoveEast()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232stopMoveWest()), this, SLOT(LXstopMoveWest()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232stopMoveNorth()), this, SLOT(LXstopMoveNorth()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232stopMoveSouth()), this, SLOT(LXstopMoveSouth()),Qt::QueuedConnection); // LX 200 handbox commands
    connect(this->lx200port,SIGNAL(RS232stopMotion()), this, SLOT(LXstopMotion()),Qt::QueuedConnection); // total stop of all motion by LX 200
    connect(this->lx200port,SIGNAL(RS232guideSpeed()), this, SLOT(LXslowSpeed()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232centerSpeed()), this, SLOT(LXslowSpeed()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232findSpeed()), this, SLOT(LXhiSpeed()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232gotoSpeed()), this, SLOT(LXhiSpeed()),Qt::QueuedConnection); // LX 200 knows four speeds, we only know 2 - sidereal correction and fast motion
    connect(this->lx200port,SIGNAL(RS232sync()),this,SLOT(LXsyncMount()),Qt::QueuedConnection); // LX 200 sync
    connect(this->lx200port,SIGNAL(RS232slew()),this,SLOT(LXslewMount()),Qt::QueuedConnection); // LX 200 slew
    connect(this->lx200port,SIGNAL(RS232CommandReceived()),this, SLOT(logLX200IncomingCmds()),Qt::QueuedConnection); // write incoming command from LX 200 to log
    connect(this->lx200port,SIGNAL(RS232RASent()),this, SLOT(logLX200OutgoingCmdsRA()),Qt::QueuedConnection); // receive RA from LX 200 and log it
    connect(this->lx200port,SIGNAL(RS232DeclSent()),this, SLOT(logLX200OutgoingCmdsDecl()),Qt::QueuedConnection); // receive decl from LX 200 and log it
    connect(this->lx200port,SIGNAL(RS232CommandSent()),this, SLOT(logLX200OutgoingCmds()),Qt::QueuedConnection); // write outgoing command from LX 200 to log
    connect(ui->cbLXSimpleNumbers, SIGNAL(released()),this, SLOT(LXSetNumberFormatToSimple())); // switch between simple and complex LX 200 format
    connect(this->camera_client,SIGNAL(imageAvailable()),this,SLOT(displayGuideCamImage()),Qt::QueuedConnection); // display image from ccd if one was received from INDI; also takes care of autoguiding. triggered by signal
    connect(this->camera_client,SIGNAL(messageFromINDIAvailable()),this,SLOT(handleServerMessage()),Qt::QueuedConnection); // display messages from INDI if signal was received
    connect(this->guiding,SIGNAL(guideImagePreviewAvailable()),this,SLOT(displayGuideStarPreview())); // handle preview of the processed guidestar image
    connect(this->ui->pbTrainAxes, SIGNAL(clicked()),this, SLOT(calibrateAutoGuider()));
    this->StepperDriveRA->stopDrive();
    this->StepperDriveDecl->stopDrive(); // just to kill all jobs that may lurk in the muproc ...
}
//------------------------------------------------------------------
// dewstructor - hopefully kills all local and global instances
MainWindow::~MainWindow() {
    delete StepperDriveRA;
    delete StepperDriveDecl;
    delete timer;
    delete textEntry;
    delete lx200port;
    delete g_AllData;
    delete camImg;
    delete guideStarPrev;
    delete ui;
    exit(0);
}

//------------------------------------------------------------------
// the main event queue, triggered by this->timer
void MainWindow::updateReadings() {
    qint64 topicalTime; // g_AllData contains an monotonic global timer that is reset if a sync occcurs
    double relativeTravelRA, relativeTravelDecl,totalGearRatio; // a few helpers

    if (this->lx200IsOn) { // check the serial port for LX 200 commands
        if (lx200port->getPortState() == 1) {
            lx200port->getDataFromSerialPort();
        }
    }
    if (this->mountMotion.RATrackingIsOn == true) { // standard mode - mount compensates for earth motion
        topicalTime = g_AllData->getTimeSinceLastSync() - this->mountMotion.RAtrackingElapsedTimeInMS; // check the monotonic timer
        this->mountMotion.RAtrackingElapsedTimeInMS+=topicalTime; // total time elapsed in tracking mode
        totalGearRatio = g_AllData->getGearData(0)*g_AllData->getGearData(1)*g_AllData->getGearData(2); // gear ratio in RA
        relativeTravelRA= this->StepperDriveRA->getKinetics(3)*topicalTime*g_AllData->getGearData(3)/
                (1000.0*g_AllData->getGearData(8)*totalGearRatio); // compute travel in decimal degrees
        g_AllData->incrementActualScopePosition(relativeTravelRA, 0.0); // update position in global struct on RA
    }
    if (this->mountMotion.RADriveIsMoving == true) { // mount moves at non-sidereal rate - but not in GOTO
        topicalTime = g_AllData->getTimeSinceLastSync() - this->mountMotion.RAMoveElapsedTimeInMS;
        this->mountMotion.RAMoveElapsedTimeInMS+=topicalTime;
        totalGearRatio = g_AllData->getGearData(0)*g_AllData->getGearData(1)*g_AllData->getGearData(2);
        relativeTravelRA=this->mountMotion.RADriveDirection*
                this->StepperDriveRA->getKinetics(3)*topicalTime*g_AllData->getGearData(3)/
                (1000.0*g_AllData->getGearData(8)*totalGearRatio);
        g_AllData->incrementActualScopePosition(relativeTravelRA, 0.0);  // same as above - compute travel in decimal degrees and update it
        if (this->StepperDriveRA->hasHBoxSlewEnded() == true) {
            // this is a little bit strange; handboxslew is 180 degrees maximum. if this occurs,
            // it has to be handled like pressing the stop button
            this->mountMotion.RADriveIsMoving = false;
            while (!futureStepperBehaviourRA.isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
            }
            if (this->mountMotion.RATrackingIsOn == false) {
                this->setControlsForRATravel(true);
            }
            this->startRATracking();
            ui->pbRAMinus->setEnabled(1);
            ui->pbRAPlus->setEnabled(1);
            ui->rbCorrSpeed->setEnabled(true);
            ui->rbMoveSpeed->setEnabled(true);
            ui->pbPGRAMinus->setEnabled(true);
            ui->pbPGRAPlus->setEnabled(true);
            ui->pbPGDecMinus->setEnabled(true);
            ui->pbPGDecPlus->setEnabled(true);
            if (ui->rbMoveSpeed->isChecked()==false) {
                ui->sbMoveSpeed->setEnabled(true);
            }
        } // a lot of code for an unlikely situation. after 180 degrees, the mount simply resumes tracking
    }

    if (this->mountMotion.DeclDriveIsMoving == true) { // now, the declination drive is also active; it does not track, therefore we have a copy of the above section, more or less
        topicalTime = g_AllData->getTimeSinceLastSync() - this->mountMotion.DeclMoveElapsedTimeInMS;
        this->mountMotion.DeclMoveElapsedTimeInMS+=topicalTime;
        totalGearRatio = g_AllData->getGearData(4)*g_AllData->getGearData(5)*g_AllData->getGearData(6);
        relativeTravelDecl= this->mountMotion.DeclDriveDirection*
                this->StepperDriveDecl->getKinetics(3)*topicalTime*g_AllData->getGearData(7)/
                (1000.0*g_AllData->getGearData(8)*totalGearRatio);
        g_AllData->incrementActualScopePosition(0.0, relativeTravelDecl); // update the declination position
        if (this->StepperDriveDecl->hasHBoxSlewEnded() == true) {
            // same as above; end of handbox slew of 180 degrees has to be handled like pressing a stop button
            this->mountMotion.DeclDriveIsMoving = false;
            while (!futureStepperBehaviourDecl.isFinished()) {
                    QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
            }
            ui->pbPGDecMinus->setEnabled(true);
            ui->pbPGDecPlus->setEnabled(true);
            ui->pbPGRAMinus->setEnabled(true);
            ui->pbPGRAPlus->setEnabled(true);
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

    if ((this->mountMotion.GoToIsActiveInRA==true) || (this->mountMotion.GoToIsActiveInDecl==true)) { // the mount is slewing. slew is not complete as either RA or decl are in slew mode - or both
        ui->lcdGotoTime->display(round((this->gotoETA-this->elapsedGoToTime->elapsed())*0.001));
        if (this->mountMotion.GoToIsActiveInRA==true) {
            topicalTime = g_AllData->getTimeSinceLastSync() - this->mountMotion.RAGoToElapsedTimeInMS;
            this->mountMotion.RAGoToElapsedTimeInMS+=topicalTime;
            totalGearRatio = g_AllData->getGearData(0)*g_AllData->getGearData(1)*g_AllData->getGearData(2);
            relativeTravelRA=this->mountMotion.RADriveDirection*
                    this->approximateGOTOSpeedRA*topicalTime*g_AllData->getGearData(3)/
                    (1000.0*g_AllData->getGearData(8)*totalGearRatio);
            g_AllData->incrementActualScopePosition(relativeTravelRA, 0.0);
        }
        if (this->mountMotion.GoToIsActiveInDecl==true) {
            topicalTime = g_AllData->getTimeSinceLastSync() - this->mountMotion.DeclGoToElapsedTimeInMS;
            this->mountMotion.DeclGoToElapsedTimeInMS+=topicalTime;
            totalGearRatio = g_AllData->getGearData(4)*g_AllData->getGearData(5)*g_AllData->getGearData(6);
            relativeTravelDecl= this->mountMotion.DeclDriveDirection*
                    this->approximateGOTOSpeedDecl*topicalTime*g_AllData->getGearData(7)/
                    (1000.0*g_AllData->getGearData(8)*totalGearRatio);
            g_AllData->incrementActualScopePosition(0.0, relativeTravelDecl);
        }
    } else {
        if ((this->mountMotion.RATrackingIsOn == false) &&
                (this->mountMotion.RADriveIsMoving == false) &&
                (this->mountMotion.DeclDriveIsMoving == false)) {
                    g_AllData->incrementActualScopePosition(0.0,0.0);
                    // the scope is at rest, but the hour angle still
                    // needs to be updated ...
        }
    } // slew has ended ...
    ui->leHourAngle->setText(textEntry->number(g_AllData->getActualScopePosition(0),'f',5));
    ui->leDecl->setText(textEntry->number(g_AllData->getActualScopePosition(1),'f',5));
    // finally, the actual scope position is updated in the GUI
}

//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// routines for basic stepper operation
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// the most important routine - is compensates for earth motion, sets all flags and disables all GUI elements that can interefere
void MainWindow::startRATracking(void) {

    this->setControlsForRATracking(false);
    ui->rbCorrSpeed->setEnabled(true);
    ui->rbMoveSpeed->setEnabled(true);
    if (ui->rbMoveSpeed->isChecked()==false) {
        ui->sbMoveSpeed->setEnabled(true);
    }
    this->StepperDriveRA->stopDrive();
    this->mountMotion.RATrackingIsOn = true;
    ui->pbStartTracking->setEnabled(0);
    ui->pbStopTracking->setEnabled(1);
    this->mountMotion.RAtrackingElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
    this->futureStepperBehaviourRATracking=QtConcurrent::run(this->StepperDriveRA, &QStepperPhidgetsRA::startTracking);
}

//------------------------------------------------------------------
// stops earthtracking. has to be called prior to all other commands issued to the RA-stepper
void MainWindow::stopRATracking(void) {

    this->setControlsForRATracking(true);
    ui->pbStartTracking->setEnabled(1);
    ui->pbStopTracking->setEnabled(0);
    this->StepperDriveRA->stopDrive();
    while (!this->futureStepperBehaviourRATracking.isFinished()) {
    } // wait till the RA-tracking thread has died ...
    this->mountMotion.RATrackingIsOn = false;
}

//------------------------------------------------------------------
// synchronizes the mount to given coordinates and sets the monotonic timer to zero
void MainWindow::syncMount(void)
{
    if (this->StepperDriveRA->getStopped() == false) { // stop tracking
        this->stopRATracking();
    }
    if (this->mountMotion.DeclDriveIsMoving == true) {
        this->mountMotion.DeclDriveIsMoving=false;
        this->StepperDriveDecl->stopDrive();
        while (!futureStepperBehaviourDecl.isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
        }
    } // stop the declination drive as well ...
    g_AllData->setSyncPosition(this->ra, this->decl);
    // convey right ascension and declination to the global parameters;
    // a microtimer starts ...
    this->startRATracking(); // start tracking again
    ui->pbGoTo->setEnabled(true); // enable GOTO as we now have a reference position
    this->MountWasSynced = true; // set a global flag
}
//---------------------------------------------------------------------
// that one handles GOTO-commands. it blocks signals and leaves when the destination is reached ...
void MainWindow::startGoToObject(void) {
    double travelRA, travelDecl, speedFactorRA, speedFactorDecl,TRamp, SRamp,
            SAtFullSpeed, TAtFullSpeed, earthTravelDuringGOTOinMSteps,
            convertDegreesToMicrostepsDecl,convertDegreesToMicrostepsRA; // variables for assessing travel time and so on
    float targetRA, targetDecl; // desired ra and decl in decimal degrees
    qint64 timestampGOTOStarted, timeDifference, timeTaken; // various time stamps
    qint64 timeEstimatedInRAInMS = 0; // estimate for travel time in RA [ms]
    qint64 timeEstimatedInDeclInMS = 0; // estimate for travel time in Decl [ms]
    long int RASteps, DeclSteps,corrsteps; // microsteps for travel plus a correction measure
    int timeForProcessingEventQueue = 100; // should be the same as the time for the event queue given in this->timer
    bool RAtakesLonger, shortSlew; // two flags. RAtakes longer = true: RAtravel longer than Decl travel. short sles of a few seconds are carried out one after another to avaid timing problems
    bool RARideIsDone; // a flag set to true when slew in RA is done

    blockSignals(true); // block sync and slew signals during slew
    ui->pbGoTo->setEnabled(false); // disable pushbutton for GOTO
    this->terminateAllMotion(); // stop the drives
    this->setControlsForGoto(false); // set some controls on disabled
    ui->pbStartTracking->setEnabled(false); // tracking button is disabled
    shortSlew=false; // we do not know how long the slew takes, so this flag is false
    timeDifference=0; // difference between estimated travcel and real travel in RA - needed for correction after slew

        // determine the travel to be taken based on steps, acceleration and end velocity
    travelRA=((g_AllData->getActualScopePosition(0))+0.0041780746*g_AllData->getTimeSinceLastSync()/1000.0)-this->ra;
    travelDecl=this->decl-g_AllData->getActualScopePosition(1); // travel in both axes based on current position
    targetRA = this->ra;
    targetDecl = this->decl; // destination
    if (travelRA < 0) {
        this->mountMotion.RADriveDirection = -1;
    } else {
        this->mountMotion.RADriveDirection = 1;
    } // determine direction in RA
    if (travelDecl < 0) {
        this->mountMotion.DeclDriveDirection = -1;
    } else {
        this->mountMotion.DeclDriveDirection = 1;
    } // determine direction in declination
    speedFactorDecl=ui->sbGoToSpeed->value();
    speedFactorRA=ui->sbGoToSpeed->value(); // set the drive speed to GOTO speed according to spinbox in GUI
    convertDegreesToMicrostepsDecl=1.0/g_AllData->getGearData(7)*g_AllData->getGearData(8)*
            g_AllData->getGearData(4)*g_AllData->getGearData(5)*g_AllData->getGearData(6);
    DeclSteps=round(fabs(travelDecl)*convertDegreesToMicrostepsDecl); // determine the number of microsteps necessary to reach target. direction is already given and unimportant here ...
    convertDegreesToMicrostepsRA=1.0/g_AllData->getGearData(3)*g_AllData->getGearData(8)*
            g_AllData->getGearData(0)*g_AllData->getGearData(1)*g_AllData->getGearData(2);
    RASteps=round(fabs(travelRA)*convertDegreesToMicrostepsRA); // determine the number of microsteps necessary to reach target. direction is already given and unimportant here ...

    TRamp = (this->StepperDriveDecl->getKinetics(3)*(speedFactorDecl))/this->StepperDriveDecl->getKinetics(2);// time needed until drive reaches full speed - vel/acc ...
    SRamp = 0.5*this->StepperDriveDecl->getKinetics(2)*TRamp*TRamp; // travel in microsteps until full speed is reached
    SAtFullSpeed = DeclSteps-2.0*SRamp; // travel after acceleration and before de-acceleration
    if (SAtFullSpeed < 0) {
        TAtFullSpeed=sqrt(DeclSteps/this->StepperDriveDecl->getKinetics(2));// if the travel is so short that full speed cannot be reached: consider a ramp that stops at the end of travel
        timeEstimatedInDeclInMS = (TAtFullSpeed)*1000+timeForProcessingEventQueue; // time in microseconds estimated for Declination-Travel
    } else {
        TAtFullSpeed = SAtFullSpeed/(this->StepperDriveDecl->getKinetics(3)*speedFactorDecl);
        timeEstimatedInDeclInMS = (TAtFullSpeed+2.0*TRamp)*1000+timeForProcessingEventQueue; // time in microseconds estimated for Declination-Travel
    }

        // Now repeat that computation for the RA drive
    TRamp = (this->StepperDriveRA->getKinetics(3)*(speedFactorRA))/this->StepperDriveRA->getKinetics(2);
    SRamp = 0.5*this->StepperDriveRA->getKinetics(2)*TRamp*TRamp;
    SAtFullSpeed = RASteps-2.0*SRamp;
    if (SAtFullSpeed < 0) {
        TAtFullSpeed=sqrt(RASteps/this->StepperDriveRA->getKinetics(2));
        timeEstimatedInRAInMS = (TAtFullSpeed)*1000+timeForProcessingEventQueue;
    } else {
        TAtFullSpeed = SAtFullSpeed/(this->StepperDriveRA->getKinetics(3)*speedFactorRA);
        timeEstimatedInRAInMS = (TAtFullSpeed+2.0*TRamp)*1000+timeForProcessingEventQueue;
    }

    earthTravelDuringGOTOinMSteps=(0.0041780746*((double)timeEstimatedInRAInMS)/1000.0)*
            convertDegreesToMicrostepsRA; // determine the addition of earth travel in sideral time into account

        // compensate for earth travel in fwd or bwd direction
    if (this->mountMotion.RADriveDirection == 1) {
        RASteps=RASteps+earthTravelDuringGOTOinMSteps;
    } else {
        RASteps=RASteps-earthTravelDuringGOTOinMSteps;
    }
    timeEstimatedInRAInMS = RASteps/((double)this->StepperDriveRA->getKinetics(3)*(speedFactorRA))*1000; // correct RA travel time for this additional motion

        // find out which drive is longer in GOTO mode
    if (timeEstimatedInDeclInMS > timeEstimatedInRAInMS) {
        gotoETA = timeEstimatedInDeclInMS; // time for total slew
        RAtakesLonger=false;
    } else {
        gotoETA = timeEstimatedInRAInMS; // time for total slew
        RAtakesLonger=true;
    }
        // if one of the travels is less than 5 seconds - carry the travel out one after another to avoid timing issues
    if ((timeEstimatedInDeclInMS < 5000) || (timeEstimatedInRAInMS < 5000)) {
        gotoETA = timeEstimatedInDeclInMS+timeEstimatedInRAInMS; // time for total slew
        shortSlew=true; // in this case,
    }

    this->approximateGOTOSpeedRA=RASteps/(timeEstimatedInRAInMS/1000.0); // for LX 200 display, a mean speed during GOTO not taking ramps into account is computed
    this->approximateGOTOSpeedDecl=DeclSteps/(timeEstimatedInDeclInMS/1000.0); // same as above
    ui->lcdGotoTime->display(round(gotoETA/1000.0)); // determined the estimated duration of the GoTo - Process and display it in the GUI. it is reduced in the event queue

    QCoreApplication::processEvents(QEventLoop::AllEvents, timeForProcessingEventQueue); // just make sure that events are processed ...

        // let the games begin ... GOTO is ready to start ...
    RARideIsDone = false; // RA slew not finished yet ...
    this->elapsedGoToTime->start(); // a second timer in the class to measure the time elapsed during goto - needed for updates in the event queue

    if (shortSlew == true) { // is the target is nearby, carry out the slews one after another ...
        futureStepperBehaviourDecl_GOTO =QtConcurrent::run(this->StepperDriveDecl,&QStepperPhidgetsDecl::travelForNSteps,DeclSteps,this->mountMotion.DeclDriveDirection,speedFactorDecl,0);
        while (!futureStepperBehaviourDecl.isStarted()) { // wait for thread to start
        }
        this->mountMotion.GoToIsActiveInDecl=true;
        timestampGOTOStarted = g_AllData->getTimeSinceLastSync(); // set a global timestamp
        this->mountMotion.DeclGoToElapsedTimeInMS=g_AllData->getTimeSinceLastSync();
        while (!futureStepperBehaviourDecl_GOTO.isFinished()) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, timeForProcessingEventQueue);
            if (this->mountMotion.emergencyStopTriggered==true) { // if the emergency button is pressed, terminate routine immediately
                this->mountMotion.emergencyStopTriggered=false;
                return;
            }
        }
        this->mountMotion.GoToIsActiveInDecl=false; // goto in Decl is now done, so start travel in RA. set also the flag for decl-goto ...
        futureStepperBehaviourRA_GOTO =QtConcurrent::run(this->StepperDriveRA,&QStepperPhidgetsRA::travelForNSteps,RASteps,this->mountMotion.RADriveDirection,speedFactorRA,false);
        while (!futureStepperBehaviourRA.isStarted()) { // wait for thread to start
        }
        this->mountMotion.GoToIsActiveInRA=true;
        this->mountMotion.RAGoToElapsedTimeInMS=g_AllData->getTimeSinceLastSync(); // set a global timestamp
        while (!futureStepperBehaviourRA_GOTO.isFinished()) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, timeForProcessingEventQueue);
            if (this->mountMotion.emergencyStopTriggered==true) { // if the emergency button is pressed, terminate routine immediately
                this->mountMotion.emergencyStopTriggered=false;
                return;
            }
        } // RA travel is done here
        timeTaken = g_AllData->getTimeSinceLastSync()-timestampGOTOStarted; // determine real time of GOTO according to monotonic timer
        timeDifference = timeTaken-timeEstimatedInRAInMS; // determine the difference between estimated and real travel
        this->startRATracking(); // start earth motion compensation
        ui->pbStopTracking->setDisabled(true); // set GUI to tracking operation
        this->mountMotion.GoToIsActiveInRA=false; // flag on RA-GOTO set to false - important for event queue
    } else { // carry out the slews parallel
        futureStepperBehaviourRA_GOTO =QtConcurrent::run(this->StepperDriveRA,&QStepperPhidgetsRA::travelForNSteps,RASteps,this->mountMotion.RADriveDirection,speedFactorRA,false);
        while (!futureStepperBehaviourRA.isStarted()) {
        }
        this->mountMotion.GoToIsActiveInRA=true;
        this->mountMotion.RAGoToElapsedTimeInMS=g_AllData->getTimeSinceLastSync();
        timestampGOTOStarted = g_AllData->getTimeSinceLastSync();
        futureStepperBehaviourDecl_GOTO =QtConcurrent::run(this->StepperDriveDecl,&QStepperPhidgetsDecl::travelForNSteps,DeclSteps,this->mountMotion.DeclDriveDirection,speedFactorDecl,0);
        while (!futureStepperBehaviourDecl.isStarted()) {
        }
        this->mountMotion.GoToIsActiveInDecl=true;
        this->mountMotion.DeclGoToElapsedTimeInMS=g_AllData->getTimeSinceLastSync(); // now, all drives are started and timestamps were taken
        if (RAtakesLonger == true) { // then decl will finish earlier
            while (!futureStepperBehaviourRA_GOTO.isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, timeForProcessingEventQueue);
                if (futureStepperBehaviourDecl_GOTO.isFinished()) {
                    this->mountMotion.GoToIsActiveInDecl=false;
                } // declination has finished here, flag is set to false
                if (this->mountMotion.emergencyStopTriggered==true) { // if the emergency button is pressed, terminate routine immediately
                    this->mountMotion.emergencyStopTriggered=false;
                    return;
                }
            } // now, RA is done as well
            timeTaken = g_AllData->getTimeSinceLastSync()-timestampGOTOStarted;
            timeDifference = timeTaken-timeEstimatedInRAInMS; // compute difference between real and estimated travel
            this->startRATracking();
            ui->pbStopTracking->setDisabled(true);
            this->mountMotion.GoToIsActiveInRA=false; // resume standard tracking
        } else { // RA will finish earlier
            while (!futureStepperBehaviourDecl_GOTO.isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, timeForProcessingEventQueue);
                if (futureStepperBehaviourRA_GOTO.isFinished()) { // ra is finished, decl still active ...
                    this->mountMotion.GoToIsActiveInRA=false;
                    if (RARideIsDone==false) { // ok - if RA is finished FOR THE FIRST TIME ...
                        RARideIsDone=true;
                        timeTaken = g_AllData->getTimeSinceLastSync()-timestampGOTOStarted;
                        timeDifference = timeTaken-timeEstimatedInRAInMS;
                        this->startRATracking();
                        ui->pbStopTracking->setDisabled(true); // ... take care of time stamps and tracking while decl is further active
                    }
                    if (this->mountMotion.emergencyStopTriggered==true) { // emergency stop handling
                        this->mountMotion.emergencyStopTriggered=false;
                        return;
                    }
                }
            }
            if (this->mountMotion.emergencyStopTriggered==true) { // emergency stop handling
                this->mountMotion.emergencyStopTriggered=false;
                return;
            }
            this->mountMotion.GoToIsActiveInDecl=false; // declination is now done
        }
    }
    usleep(100); // just a little bit of rest :D
    this->stopRATracking(); // stop tracking, either for correction of for sync
    if (abs(timeDifference)>100) { // if the error in goto time estimation is bigger than 0.1 s, correcct in RA
        corrsteps=(0.0041780746*((double)(timeDifference))/1000.0)*convertDegreesToMicrostepsRA; // number of correction steps
        futureStepperBehaviourRA_Corr = QtConcurrent::run(this->StepperDriveRA,
                &QStepperPhidgetsRA::travelForNSteps,corrsteps, 1,10,false);
        while (!futureStepperBehaviourRA_Corr.isFinished()) {
           QCoreApplication::processEvents(QEventLoop::AllEvents, timeForProcessingEventQueue);
        }
        if (this->mountMotion.emergencyStopTriggered==true) { // emergency stop handling
            this->mountMotion.emergencyStopTriggered=false;
            return;
        }
    } // coorrection is now done as well
    this->ra=targetRA;
    this->decl=targetDecl;
    this->syncMount(); // sync the mount
    ui->lcdGotoTime->display(0); // set the LCD counter to zero again
    ui->pbGoTo->setEnabled(true);
    ui->pbStopTracking->setDisabled(false);
    this->setControlsForGoto(true);
    this->setControlsForRATravel(true); // set GUI back in base state
    blockSignals(false); // allow for signals again - this is especially important for LX200 stuff ...
    return;
}

//------------------------------------------------------------------
// handles shutdown of the program
void MainWindow::shutDownProgram() {
    this->ccdCameraIsAcquiring=false;
    sleep(ui->sbExposureTime->value());
    camera_client->sayGoodbyeToINDIServer();
    this->StepperDriveRA->stopDrive();
    delete StepperDriveRA;
    this->StepperDriveDecl->stopDrive();
    delete StepperDriveDecl;
    exit(0);
}

//---------------------------------------------------------------------
// soft stop for drives used in GOTO
void MainWindow::terminateAllMotion(void) {
    if (this->mountMotion.RADriveIsMoving == true) {
        this->mountMotion.RADriveIsMoving=false;
        this->StepperDriveRA->stopDrive();
        while (!futureStepperBehaviourRA.isFinished()) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
        }
    }
    if (this->mountMotion.DeclDriveIsMoving == true) {
        this->mountMotion.DeclDriveIsMoving=false;
        this->StepperDriveDecl->stopDrive();
        while (!futureStepperBehaviourDecl.isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
        }
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
    this->mountMotion.RATrackingIsOn = false;
    this->mountMotion.RADriveIsMoving = false;
    this->mountMotion.DeclDriveIsMoving = false;
    this->mountMotion.GoToIsActiveInRA = false;
    this->mountMotion.GoToIsActiveInDecl = false;
    while (!futureStepperBehaviourRATracking.isFinished()) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
    };
    while (!futureStepperBehaviourRA.isFinished()) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
    };
    while (!futureStepperBehaviourDecl.isFinished()) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
    };
    while (!futureStepperBehaviourRA_GOTO.isFinished()) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
    };
    while (!futureStepperBehaviourDecl_GOTO.isFinished()) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
    };
    while (!futureStepperBehaviourRA_Corr.isFinished()) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
    };
    while (!futureStepperBehaviourDecl_Corr.isFinished()) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
    };
    ui->pbStartTracking->setEnabled(true);
    ui->pbStopTracking->setEnabled(false);
    this->setControlsForGoto(true);
    ui->lcdGotoTime->display(0);
    ui->pbGoTo->setEnabled(true);
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
void MainWindow::setMaxStepperAccRA(void)
{
    double val;
    QString *leEntry;
    bool trackingWasOn;

    trackingWasOn = this->mountMotion.RATrackingIsOn;
    if (this->mountMotion.RATrackingIsOn == true) {
        this->stopRATracking();
    }
    leEntry = new QString(ui->leAMaxRA->text());
    val = (double)(leEntry->toFloat());
    this->StepperDriveRA->setStepperParams(val, 1);
    if (trackingWasOn == true) {
        this->startRATracking();
    }
    g_AllData->setDriveParams(0,1,val);
    delete leEntry;
}

//------------------------------------------------------------------
// convey acceleration to the stepper class from the GUI
void MainWindow::setMaxStepperAccDecl(void) {
    double val;
    QString *leEntry;

    leEntry = new QString(ui->leAMaxDecl->text());
    val = (double)(leEntry->toFloat());
    this->StepperDriveDecl->setStepperParams(val, 1);
    g_AllData->setDriveParams(1,1,val);
    delete leEntry;
}

//------------------------------------------------------------------
// convey current to the stepper class from the GUI
void MainWindow::setMaxStepperCurrentRA(void)
{
    double val;
    QString *leEntry;
    bool trackingWasOn;

    trackingWasOn = this->mountMotion.RATrackingIsOn;
    if (this->mountMotion.RATrackingIsOn == true) {
        this->stopRATracking();
    }
    leEntry = new QString(ui->leCurrMaxRA->text());
    val = (double)(leEntry->toFloat());
    this->StepperDriveRA->setStepperParams(val, 3);
    if (trackingWasOn == true) {
        this->startRATracking();
    }
    g_AllData->setDriveParams(0,2,val);
    delete leEntry;
}
//------------------------------------------------------------------
// convey current to the stepper class from the GUI
void MainWindow::setMaxStepperCurrentDecl(void) {
    double val;
    QString *leEntry;

    leEntry = new QString(ui->leCurrMaxDecl->text());
    val = (double)(leEntry->toFloat());
    this->StepperDriveDecl->setStepperParams(val, 3);
    g_AllData->setDriveParams(1,2,val);
    delete leEntry;
}

//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
// routines for communication with the INDI server
//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
// read the address and port from the GUI and start connect the camera to the INDI server
void MainWindow::setINDISAddrAndPort(void) {
    QString saddr;
    int sport;
    bool isServerUp = 0;

    saddr=ui->leINDIServer->text();
    sport=ui->sbINDIPort->value();
    isServerUp = camera_client->setINDIServer(saddr,sport);
    g_AllData->setINDIState(isServerUp);
    // set a global flag on the server state
    if (isServerUp== true) {
        ui->pbExpose->setEnabled(true);
        ui->cbIndiIsUp->setChecked(true);
        ui->pbGetCCDParams->setEnabled(true);
        ui->pbCCDTakeDarks->setEnabled(true);
        ui->pbCCDTakeFlats->setEnabled(true);
        ui->pbTrainAxes->setEnabled(true);
        ui->cbStoreGuideCamImgs->setEnabled(true);
        ui->pbSelectGuideStar->setEnabled(true);
    }
}

//------------------------------------------------------------------
// if a message from INDI is received by the camera class, process the signal and display it
void MainWindow::handleServerMessage(void) {
    QString *indiMesg;

    indiMesg=new QString(camera_client->getINDIServerMessage()->toLatin1());
    if (indiMesg->isEmpty()==false) {
        ui->textEditINDIMsgs->insertPlainText(indiMesg->toLatin1());
        ui->textEditINDIMsgs->insertPlainText("\n");
    }
    delete indiMesg;
}
//------------------------------------------------------------------
// deploy a system command to start an INDI server locally with standard parameters.
// type of server is defined by radiobuttons, only QHY until now supported ...
void MainWindow::deployINDICommand(void) {
    int retval=0;

    if (ui->rbQHYINDI->isChecked()== true) {
        retval = system("indiserver -v -m 100 indi_qhy_ccd &");
    }
    ui->pbStartINDIServer->setEnabled(false);
}

//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
// routines for handling the ccd - camera
//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
// send an exposure time to the camera and tell INDI to take an image
// an event is triggered once these data were received
void MainWindow::takeSingleCamShot(void) {
   int exptime;

   exptime = (ui->sbExposureTime->value());
   camera_client->takeExposure(exptime);
}

//------------------------------------------------------------------
// prepare GUI for taking images; once an image was received, a
// new one is requested in "displayGuideCamImage"
void MainWindow::startCCDAcquisition(void) {
    this->ccdCameraIsAcquiring=true;
    ui->pbExpose->setEnabled(false);
    ui->pbStopExposure->setEnabled(true);
    takeSingleCamShot();
}

//------------------------------------------------------------------
// set a flag so that no new image is requested in "displayGuideCamImage"
void MainWindow::stopCCDAcquisition(void) {
    this->ccdCameraIsAcquiring=false;
    ui->pbStopExposure->setEnabled(false);
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
// takes care of precessing the preview image. also polls new images
// if the appropriate flag is set
void MainWindow::displayGuideCamImage(void) {
    int thrshld,beta;
    float newX, newY,alpha;
    bool medianOn;

    thrshld = ui->hsThreshold->value();
    alpha = ui->hsIContrast->value()/100.0;
    beta = ui->hsIBrightness->value();
    medianOn=ui->cbMedianFilter->isChecked(); // get parameters for guidestar-processing from GUI
    if (g_AllData->getINDIState() == true) { // ... if the camera class is connected to the INDI server ...
        this->updateCameraImage(); // get a pixmap from the camera class
        if (this->ccdCameraIsAcquiring==true) { // if the flag for taking another one is true ...
            this->takeSingleCamShot(); // ... request another one from INDI
            if (g_AllData->getGuideScopeFlags(3) == true) { // autoguider is calibrating
                g_AllData->setGuideScopeFlags(true,5); // in calibration, this camera image is to be used
            }
            if (this->guidingIsActive==true) { // if autoguiding is active ...
                this->guiding->doGuideStarImgProcessing(thrshld,medianOn,alpha,beta,this->guidingFOVFactor); // ... process the guide star subimage
                newX = g_AllData->getInitialStarPosition(2);
                newY = g_AllData->getInitialStarPosition(3); // the star centroid found in "doGuideStarImgProcessing" was stored in the global struct ...
                correctGuideStarPosition(newX,newY); // ... and is used to correct the position
            }
        } else {
           ui->pbExpose->setEnabled(true); // if acquisition is disabled, set the GUI so that it can be enabled
        }
    }
}
//------------------------------------------------------------------
// retrieve a pixmap for display from the camera class
void MainWindow::updateCameraImage(void) {
    camImg=camera_client->getScaledPixmapFromCamera();
    this->camView->addBgImage(*camImg);
}
//------------------------------------------------------------------
// retrieve parameters for the CCD from the camera class
 bool MainWindow::getCCDParameters(void) {
    bool retrievalSuccess;
    QString letxt;
    float psx,psy;
    int fsx,fsy;

    retrievalSuccess = camera_client->getCCDParameters();
    if (retrievalSuccess==1) {
        psx=g_AllData->getCameraPixelSize(0);
        psy=g_AllData->getCameraPixelSize(1);
        fsx=(int)g_AllData->getCameraChipPixels(0);
        fsy=(int)g_AllData->getCameraChipPixels(1);
        letxt=QString::number((double)psx,'g',2);
        ui->lePixelSizeX->setText(letxt);
        letxt=QString::number((double)psy,'g',2);
        ui->lePixelSizeY->setText(letxt);
        letxt=QString::number(fsx);
        ui->leFrameSizeX->setText(letxt);
        letxt=QString::number(fsy);
        ui->leFrameSizeY->setText(letxt);
    }
    return retrievalSuccess;
}

 //------------------------------------------------------------------
 // store data on the ccd from the GUI to the global data and to the .tsp file ...
 void MainWindow::storeCCDData(void)
 {
     float psx,psy;
     int ccdw, ccdh;
     QString *leEntry;

     leEntry = new QString(ui->lePixelSizeX->text());
     psx=leEntry->toFloat();
     leEntry->clear();
     leEntry->append(ui->lePixelSizeY->text());
     psy=leEntry->toFloat();
     leEntry->clear();
     leEntry->append(ui->leFrameSizeX->text());
     ccdw=leEntry->toInt();
     leEntry->clear();
     leEntry->append(ui->leFrameSizeY->text());
     ccdh=leEntry->toInt();
     delete leEntry;
     g_AllData->setCameraParameters(psx,psy,ccdw,ccdh);
     g_AllData->storeGlobalData();
 }

//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
// routines for auto guiding
//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
// THIS IS NOT DONE - correct guide star position here ....
double MainWindow::correctGuideStarPosition(float cx, float cy) {
    float residualX, residualY;

    residualX=(this->guideStarPosition.centrX - cx)*this->guiding->getArcSecsPerPix(0);
    residualY=(this->guideStarPosition.centrY - cy)*this->guiding->getArcSecsPerPix(1);
    ui->leXDev->setText(textEntry->number(residualX));
    ui->leYDev->setText(textEntry->number(residualY));
    qDebug() << "Relative Move" << residualX << residualY;
     // do something here, and get a new centroid ...
    this->guideStarPosition.centrX = cx;
    this->guideStarPosition.centrY = cy;
    return 0.0;
}

//------------------------------------------------------------------
// calibrate the system. the selected star is located and three
// pulse guide commands in each direction and back are carried out.
// the pixel/ms is then evaluated for each direction - UNDER CONSTRUCTION
void MainWindow::calibrateAutoGuider(void) {
    int pulseDuration;
    double currentCentroid[2], initialCentroid[2], raPlusUnitVector[2],arcsecPPix[2],ccdFOVInArcSec,
            travelPerMSInRACorr,travelPerMSInDeclCorr,travelTimeInMSForOnePixRA,lengthOfTravel;
    int thrshld,beta;
    float alpha;
    bool medianOn;
    QString statMesg;

    setControlsForAutoguiderCalibration(false);
    ui->teCalibrationStatus->appendPlainText("Entering calibration...");
    thrshld = ui->hsThreshold->value();
    alpha = ui->hsIContrast->value()/100.0;
    beta = ui->hsIBrightness->value();
    medianOn=ui->cbMedianFilter->isChecked(); // get parameters for guidestar-processing from GUI
    raPlusUnitVector[0]=0.0;
    raPlusUnitVector[1]=0.0; // initialize a vector of length 1 that gives the direction of RA+ travel on the chip
    arcsecPPix[0] = this->guiding->getArcSecsPerPix(0);
    arcsecPPix[1] = this->guiding->getArcSecsPerPix(1); // get the ratio "/pixel from the guiding class
    if (g_AllData->getCameraChipPixels(0)*arcsecPPix[0] > g_AllData->getCameraChipPixels(1)*arcsecPPix[1]) {
        ccdFOVInArcSec=g_AllData->getCameraChipPixels(1)*arcsecPPix[1];
    } else {
        ccdFOVInArcSec=g_AllData->getCameraChipPixels(0)*arcsecPPix[0];
    } // determine the smaller dimension of the chip to determine the possible travel over the chip in arcseconds...
    travelPerMSInRACorr=0.001*(3600.0)*g_AllData->getDriveParams(0,0)*(g_AllData->getGearData(3)/g_AllData->getGearData(8))/
        (g_AllData->getGearData(0)*g_AllData->getGearData(1)*g_AllData->getGearData(2));
    travelPerMSInDeclCorr=0.001*(3600.0)*g_AllData->getDriveParams(1,0)*(g_AllData->getGearData(7)/g_AllData->getGearData(8))/
        (g_AllData->getGearData(4)*g_AllData->getGearData(5)*g_AllData->getGearData(6));
        // computed the travel in arcseconds per millisecond pulse guiding
    travelTimeInMSForOnePixRA=arcsecPPix[0]/travelPerMSInRACorr;
    statMesg.append("FOV: ");
    statMesg.append(QString::number((double)(ccdFOVInArcSec/60.0),'g',2));
    statMesg.append("'");
    ui->teCalibrationStatus->appendPlainText(statMesg);
    statMesg.clear();
    ui->teCalibrationStatus->appendPlainText("Time for 1 pix (RA):");
    statMesg.append(QString::number((double)travelTimeInMSForOnePixRA,'g',2));
    statMesg.append(" ms");
    ui->teCalibrationStatus->appendPlainText(statMesg);
    statMesg.clear();

    // now determine the direction of RA Travel as a unit vector; travel for 10 pix ...
    pulseDuration = 10*travelTimeInMSForOnePixRA;
    ui->teCalibrationStatus->appendPlainText("Waiting for image...");
    this->waitForCalibrationImage();
    this->guiding->doGuideStarImgProcessing(thrshld,medianOn,alpha,beta,this->guidingFOVFactor); // ... process the guide star subimage
    initialCentroid[0] = g_AllData->getInitialStarPosition(2);
    initialCentroid[1] = g_AllData->getInitialStarPosition(3); // first centroid before slew
    ui->sbPulseGuideDuration->setValue(pulseDuration); // set the duration for the 10 pixel slew
    ui->teCalibrationStatus->appendPlainText("Slewing 10 pix ...");
    this->raPGFwd();
    ui->teCalibrationStatus->appendPlainText("Waiting for image...");
    this->waitForCalibrationImage();
    this->guiding->doGuideStarImgProcessing(thrshld,medianOn,alpha,beta,this->guidingFOVFactor); // ... process the guide star subimage
    currentCentroid[0] = g_AllData->getInitialStarPosition(2);
    currentCentroid[1] = g_AllData->getInitialStarPosition(3); // centroid after slew
    raPlusUnitVector[0] = currentCentroid[0]-initialCentroid[0];
    raPlusUnitVector[1] = currentCentroid[1]-initialCentroid[1];
    lengthOfTravel=sqrt(raPlusUnitVector[0]*raPlusUnitVector[0]+raPlusUnitVector[1]*raPlusUnitVector[1]);
    raPlusUnitVector[0]/= lengthOfTravel;
    raPlusUnitVector[1]/= lengthOfTravel; // computed the direction vector of length one for RA+ travel
    qDebug() << "Unit vector" << raPlusUnitVector[0] << raPlusUnitVector[1];
    ui->teCalibrationStatus->appendPlainText("Slewing back...");
    this->raPGBwd(); // going back to initial position
    ui->teCalibrationStatus->appendPlainText("Direction RA:");
    statMesg = QString::number((double)raPlusUnitVector[0],'g',3);
    statMesg.append("/");
    statMesg.append(QString::number((double)raPlusUnitVector[1],'g',3));
    ui->teCalibrationStatus->appendPlainText(statMesg);
    statMesg.clear();

    // now get the centroid again and do a bigger slew in RA plus direction for a more exact position determination
    ui->teCalibrationStatus->appendPlainText("Waiting for image...");
    this->waitForCalibrationImage();
    this->guiding->doGuideStarImgProcessing(thrshld,medianOn,alpha,beta,this->guidingFOVFactor); // ... process the guide star subimage
    initialCentroid[0] = g_AllData->getInitialStarPosition(2);
    initialCentroid[1] = g_AllData->getInitialStarPosition(3); // get the current centroid




    g_AllData->setGuideScopeFlags(false,3); // "calibrationIsRunning" - flag set to false
    g_AllData->setGuideScopeFlags(true,4); // "systemIsCalibrated" - flag set to true
    setControlsForAutoguiderCalibration(true);
    ui->teCalibrationStatus->appendPlainText("Calibration is finished...");
}

//------------------------------------------------------------------
// a subroutine that waits for two images from the ccd. needed in
// "calibrateAutoGuider"
void MainWindow::waitForCalibrationImage(void) {
    g_AllData->setGuideScopeFlags(true,3); // "calibrationIsRunning" - flag set to true
    g_AllData->setGuideScopeFlags(false,5); // "calibrationImageReceived" - flag is set to false
    while (g_AllData->getGuideScopeFlags(5) == false) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
    }
    g_AllData->setGuideScopeFlags(false,5); // "calibrationImageReceived" - flag is set to false
    while (g_AllData->getGuideScopeFlags(5) == false) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
    }
    // read two images to make sure that an image is not taken during slew ...
    g_AllData->setGuideScopeFlags(false,3); // "calibrationIsRunning" - flag set to false
}

//------------------------------------------------------------------
// prepare the GUI and the flags for autoguiding; the actual work is done
// in "displayGuideCamImage" and "correctGuideStarPosition" ...
void MainWindow::doAutoGuiding(void) {

    if (this->guidingIsActive==false) {
        g_AllData->setGuideScopeFlags(true,2);
        this->guidingIsActive=true;
        this->setControlsForGuiding(false);
        // take care of disabling the gui here ...
        ui->pbGuiding->setText("Stop");
    } else {
        this->guidingIsActive=false;
        g_AllData->setGuideScopeFlags(false,2);
        ui->pbGuiding->setText("Guide");
        this->setControlsForGuiding(true);
        // enable the GUI here again ...
    }
}

//------------------------------------------------------------------
// slot for finding a guide star. the position is initially taken
// from the crosshair position stored in g_AllData.
void MainWindow::selectGuideStar(void) {
    int thrshld,beta;
    bool medianOn;
    float alpha;

    if (this->mountMotion.RATrackingIsOn==false) {
        this->startRATracking();
    } // turn on tracking if it is not running when a guide star is selected
    ui->pbGuiding->setEnabled(true);
    ui->hsThreshold->setEnabled(true);
    ui->hsIContrast->setEnabled(true);
    ui->hsIBrightness->setEnabled(true); // enable image processing controls
    medianOn=ui->cbMedianFilter->isChecked();
    thrshld = ui->hsThreshold->value();
    alpha = ui->hsIContrast->value()/100.0;
    beta = ui->hsIBrightness->value(); // get image processing parameters
    g_AllData->setGuideScopeFlags(true,1); // set flag for a selected star
    this->guiding->doGuideStarImgProcessing(thrshld,medianOn,alpha,beta,this->guidingFOVFactor);
    guideStarPosition.centrX = g_AllData->getInitialStarPosition(2);
    guideStarPosition.centrY = g_AllData->getInitialStarPosition(3); // "doGuideStarImgProcessing" stores a position in g_AllData
}

//------------------------------------------------------------------
// slot for changing the image processing controls
void MainWindow::changePrevImgProc(void) {
    int thrshld,beta;
    bool medianOn;
    float alpha;

    thrshld = ui->hsThreshold->value();
    medianOn=ui->cbMedianFilter->isChecked();
    alpha = ui->hsIContrast->value()/100.0;
    beta = ui->hsIBrightness->value();
    this->guiding->doGuideStarImgProcessing(thrshld,medianOn,alpha,beta, this->guidingFOVFactor);
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
// store guide scope focal length to .twp preference file
void MainWindow::storeGuideScopeFL(void) {
    this->changeGuideScopeFL();
    g_AllData->storeGlobalData();
}

//------------------------------------------------------------------
// reduce the guide star window to 90x90 pixels
void MainWindow::setHalfFOV(void) {
    int thrshld,beta;
    float alpha;
    bool medianOn;

    this->guidingFOVFactor=0.5;
    thrshld = ui->hsThreshold->value();
    alpha = ui->hsIContrast->value()/100.0;
    beta = ui->hsIBrightness->value();
    medianOn=ui->cbMedianFilter->isChecked();
    this->guiding->doGuideStarImgProcessing(thrshld,medianOn,alpha,beta,this->guidingFOVFactor);
}

//------------------------------------------------------------------
// reduce the guide star window to 360x360 pixels
void MainWindow::setDoubleFOV(void) {
    int thrshld,beta;
    float alpha;
    bool medianOn;

    this->guidingFOVFactor=2.0;
    thrshld = ui->hsThreshold->value();
    alpha = ui->hsIContrast->value()/100.0;
    beta = ui->hsIBrightness->value();
    medianOn=ui->cbMedianFilter->isChecked();
    this->guiding->doGuideStarImgProcessing(thrshld,medianOn,alpha,beta,this->guidingFOVFactor);
}

//------------------------------------------------------------------
// reduce the guide star window to 180x180 pixels
void MainWindow::setRegularFOV(void) {
    int thrshld,beta;
    float alpha;
    bool medianOn;

    this->guidingFOVFactor=1.0;
    thrshld = ui->hsThreshold->value();
    alpha = ui->hsIContrast->value()/100.0;
    beta = ui->hsIBrightness->value();
    medianOn=ui->cbMedianFilter->isChecked();
    this->guiding->doGuideStarImgProcessing(thrshld,medianOn,alpha,beta,this->guidingFOVFactor);
}

//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
// LX 200 related stuff
//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
// log incoming requests from LX 200
void MainWindow::logLX200IncomingCmds(void) {
    QString* lx200msg;

    if ((this->lx200IsOn==true) && (ui->cbLX200Logs->isChecked()==true)) {
        lx200msg = new QString("Incoming: ");
        lx200msg->append(this->lx200port->getLX200Command());
        ui->teLX200Data->insertPlainText(lx200msg->toLatin1());
        ui->teLX200Data->insertPlainText("\n");
        delete lx200msg;
    }
}

//------------------------------------------------------------------
// log RA commands from LX 200
void MainWindow::logLX200OutgoingCmdsRA(void) {
    QString* lx200msg;

    if ((this->lx200IsOn==true) && (ui->cbLX200Logs->isChecked()==true)) {
        lx200msg = new QString("Outgoing: ");
        lx200msg->append(this->lx200port->getLX200ResponseRA());
        ui->teLX200Data->insertPlainText(lx200msg->toLatin1());
        ui->teLX200Data->insertPlainText("\n");
        delete lx200msg;
    }
}

//------------------------------------------------------------------
// log declination commands in LX 200
void MainWindow::logLX200OutgoingCmdsDecl(void) {
    QString* lx200msg;

    if ((this->lx200IsOn==true) && (ui->cbLX200Logs->isChecked()==true)) {
        lx200msg = new QString("Outgoing: ");
        lx200msg->append(this->lx200port->getLX200ResponseDecl());
        ui->teLX200Data->insertPlainText(lx200msg->toLatin1());
        ui->teLX200Data->insertPlainText("\n");
        delete lx200msg;
    }
}
//------------------------------------------------------------------
// log outgoing commands from LX 200
void MainWindow::logLX200OutgoingCmds(void) {
    QString* lx200msg;

    if ((this->lx200IsOn==true) && (ui->cbLX200Logs->isChecked()==true)) {
        lx200msg = new QString("Outgoing: ");
        lx200msg->append(this->lx200port->getLX200Response());
        ui->teLX200Data->insertPlainText(lx200msg->toLatin1());
        ui->teLX200Data->insertPlainText("\n");
        delete lx200msg;
    }
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

    if (this->guidingIsActive==false) {
        if (this->StepperDriveRA->getStopped() == false) {
            this->stopRATracking();
        }
        if (this->mountMotion.DeclDriveIsMoving == true) {
            this->mountMotion.DeclDriveIsMoving=false;
            this->StepperDriveDecl->stopDrive();
            while (!futureStepperBehaviourDecl.isFinished()) {
                    QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
            }
        }
        this->ra = (float)(this->lx200port->getReceivedCoordinates(0));
        this->decl = (float)(this->lx200port->getReceivedCoordinates(1));
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        qDebug() << "LX Sync with" << this->ra << "and" << this->decl;
        g_AllData->setSyncPosition(this->ra, this->decl);
        // convey right ascension and declination to the global parameters;
        // a microtimer starts ...
        this->startRATracking();
        lestr = QString::number(this->ra, 'g', 8);
        ui->lineEditRA->setText(lestr);
        ui->leLX200RA->setText(lestr);
        lestr = QString::number(this->decl, 'g', 8);
        ui->lineEditDecl->setText(lestr);
        ui->leLX200Decl->setText(lestr);
        this->MountWasSynced = true;
    }
}

//---------------------------------------------------------------------
// trigger an emergency stop via LX 200
void MainWindow::LXstopMotion(void) {
    this->emergencyStop();
}

//---------------------------------------------------------------------
// slew via LX 200
void MainWindow::LXslewMount(void) {
    QString lestr;
    if (this->guidingIsActive==false) {
        if ((mountMotion.GoToIsActiveInRA==false) || (mountMotion.GoToIsActiveInDecl== false)) {
            if (this->MountWasSynced == true) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
                this->ra = (float)(this->lx200port->getReceivedCoordinates(0));
                this->decl = (float)(this->lx200port->getReceivedCoordinates(1));
                lestr = QString::number(this->ra, 'g', 8);
                ui->lineEditRA->setText(lestr);
                ui->leLX200RA->setText(lestr);
                lestr = QString::number(this->decl, 'g', 8);
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
    if (this->guidingIsActive==false) {
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

    if (this->guidingIsActive==false) {
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
    if (this->guidingIsActive==false) {
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
    if (this->guidingIsActive==false) {
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
    if (this->guidingIsActive==false) {
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
    if (this->guidingIsActive==false) {
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
    if (this->guidingIsActive==false) {
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
    if (this->guidingIsActive==false) {
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
    if (this->guidingIsActive==false) {
        ui->rbCorrSpeed->setChecked(true);
        this->setCorrectionSpeed();
    }
}

//---------------------------------------------------------------------
// motion via LX 200
void MainWindow::LXhiSpeed(void) {
    if (this->guidingIsActive==false) {
        ui->rbMoveSpeed->setChecked(true);
        this->setMoveSpeed();
    }
}

//---------------------------------------------------------------------
// change LX 200 number format
void MainWindow::LXSetNumberFormatToSimple(void) {

    if (ui->cbLXSimpleNumbers->isChecked() == true) {
        this->lx200port->setNumberFormat(true);
    } else {
        this->lx200port->setNumberFormat(false);
    }
}

//---------------------------------------------------------------------
// enable or disable the serial port
void MainWindow::switchToLX200(void) {
    if (this->lx200IsOn==false) {
        this->lx200port->openPort();
        this->lx200IsOn=true;
        ui->pbLX200Active->setText("Deactivate LX200");
        ui->cbRS232Open->setChecked(true);
    } else {
        this->lx200port->shutDownPort();
        this->lx200IsOn=false;
        ui->pbLX200Active->setText("Activate LX200");
        ui->cbRS232Open->setChecked(false);
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
        this->mountMotion.DeclMoveElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
        ui->pbPGDecMinus->setEnabled(false);
        ui->pbPGDecPlus->setEnabled(false);
        ui->pbPGRAMinus->setEnabled(false);
        ui->pbPGRAPlus->setEnabled(false);
        ui->pbDeclDown->setEnabled(0);
        this->setControlsForDeclTravel(false);
        this->mountMotion.DeclDriveIsMoving=true;
        maxDeclSteps=180.0/g_AllData->getGearData(7)*g_AllData->getGearData(8)*
                g_AllData->getGearData(4)*g_AllData->getGearData(5)*
                g_AllData->getGearData(6); // travel 180° at most
        this->mountMotion.DeclDriveDirection=1;
        futureStepperBehaviourDecl =
                QtConcurrent::run(this->StepperDriveDecl,
                &QStepperPhidgetsDecl::travelForNSteps,maxDeclSteps,
                                  this->mountMotion.DeclDriveDirection,
                                  this->mountMotion.DeclSpeedFactor,1);
        while (!futureStepperBehaviourDecl.isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        }
    } else {
        this->mountMotion.DeclDriveIsMoving=false;
        this->StepperDriveDecl->stopDrive();
        while (!futureStepperBehaviourDecl.isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        }
        ui->pbDeclDown->setEnabled(1);
        this->setControlsForDeclTravel(true);
        ui->pbPGDecMinus->setEnabled(true);
        ui->pbPGDecPlus->setEnabled(true);
        ui->pbPGRAMinus->setEnabled(true);
        ui->pbPGRAPlus->setEnabled(true);
        if (ui->rbMoveSpeed->isChecked()==false) {
            ui->sbMoveSpeed->setEnabled(true);
        } else {
            ui->sbMoveSpeed->setEnabled(false);
        }
    }
}

//--------------------------------------------------------------
void MainWindow::declinationMoveHandboxDown(void) {
    long maxDeclSteps;

    if (this->mountMotion.DeclDriveIsMoving==false){
        ui->pbPGDecMinus->setEnabled(false);
        ui->pbPGDecPlus->setEnabled(false);
        ui->pbPGRAMinus->setEnabled(false);
        ui->pbPGRAPlus->setEnabled(false);
        this->mountMotion.DeclMoveElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
        ui->pbDeclUp->setEnabled(0);
        this->setControlsForDeclTravel(false);
        this->mountMotion.DeclDriveIsMoving=true;
        this->mountMotion.DeclDriveDirection = -1;
        maxDeclSteps=180.0/g_AllData->getGearData(7)*g_AllData->getGearData(8)*
                g_AllData->getGearData(4)*g_AllData->getGearData(5)*
                g_AllData->getGearData(6); // travel 180° at most
        futureStepperBehaviourDecl =
                QtConcurrent::run(this->StepperDriveDecl,
                &QStepperPhidgetsDecl::travelForNSteps,maxDeclSteps,
                                  this->mountMotion.DeclDriveDirection,
                                  this->mountMotion.DeclSpeedFactor,1);
        while (!futureStepperBehaviourDecl.isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        }
    } else {
        this->mountMotion.DeclDriveIsMoving=false;
        this->StepperDriveDecl->stopDrive();
        while (!futureStepperBehaviourDecl.isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        }
        ui->pbPGDecMinus->setEnabled(true);
        ui->pbPGDecPlus->setEnabled(true);
        ui->pbPGRAMinus->setEnabled(true);
        ui->pbPGRAPlus->setEnabled(true);
        ui->pbDeclUp->setEnabled(1);
        this->setControlsForDeclTravel(true);
        if (ui->rbMoveSpeed->isChecked()==false) {
            ui->sbMoveSpeed->setEnabled(true);
        } else {
            ui->sbMoveSpeed->setEnabled(false);
        }
    }
}
//--------------------------------------------------------------
void MainWindow::RAMoveHandboxFwd(void) {
    long maxRASteps;
    long fwdFactor;

    if (this->mountMotion.RATrackingIsOn == true) {
        this->stopRATracking();
    }
    ui->rbCorrSpeed->setEnabled(false);
    ui->rbMoveSpeed->setEnabled(false);
    ui->sbMoveSpeed->setEnabled(false);
    ui->pbPGRAMinus->setEnabled(false);
    ui->pbPGRAPlus->setEnabled(false);
    ui->pbPGDecMinus->setEnabled(false);
    ui->pbPGDecPlus->setEnabled(false);
    if (this->mountMotion.RADriveIsMoving ==false){
        this->mountMotion.RAMoveElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
        ui->pbRAMinus->setEnabled(0);
        ui->pbStartTracking->setEnabled(0);
        ui->pbStopTracking->setEnabled(0);
        this->setControlsForRATravel(false);
        this->mountMotion.RADriveIsMoving=true;
        maxRASteps=180/g_AllData->getGearData(3)*g_AllData->getGearData(8)*
                g_AllData->getGearData(0)*g_AllData->getGearData(1)*
                g_AllData->getGearData(2); // travel 180° at most

        this->mountMotion.RADriveDirection=1;
        fwdFactor = this->mountMotion.RASpeedFactor+1; // forward motion means increase the speed
        futureStepperBehaviourRA =
                QtConcurrent::run(this->StepperDriveRA,
                &QStepperPhidgetsRA::travelForNSteps,maxRASteps,
                                  this->mountMotion.RADriveDirection,
                                  fwdFactor,true);
        while (!futureStepperBehaviourRA.isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        }
    } else {
        this->mountMotion.RADriveIsMoving=false;
        this->StepperDriveRA->stopDrive();
        while (!futureStepperBehaviourRA.isFinished()) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        }
        if (this->mountMotion.RATrackingIsOn == false) {
            this->setControlsForRATravel(true);
        }
        this->startRATracking();
        ui->pbRAMinus->setEnabled(1);
        ui->rbCorrSpeed->setEnabled(true);
        ui->rbMoveSpeed->setEnabled(true);
        ui->pbPGRAMinus->setEnabled(true);
        ui->pbPGRAPlus->setEnabled(true);
        ui->pbPGDecMinus->setEnabled(true);
        ui->pbPGDecPlus->setEnabled(true);
        if (ui->rbMoveSpeed->isChecked()==false) {
            ui->sbMoveSpeed->setEnabled(true);
        }
    }
}

//---------------------------------------------------------------------
void MainWindow::RAMoveHandboxBwd(void)
{
    long maxRASteps;
    double bwdFactor;

    if (this->mountMotion.RATrackingIsOn == true) {
        this->stopRATracking();
    }
    ui->rbCorrSpeed->setEnabled(false);
    ui->rbMoveSpeed->setEnabled(false);
    ui->sbMoveSpeed->setEnabled(false);
    ui->pbPGRAMinus->setEnabled(false);
    ui->pbPGRAPlus->setEnabled(false);
    ui->pbPGDecMinus->setEnabled(false);
    ui->pbPGDecPlus->setEnabled(false);
    if (this->mountMotion.RADriveIsMoving ==false){
        this->mountMotion.RAMoveElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
        ui->pbRAPlus->setEnabled(0);
        setControlsForRATravel(false);
        ui->pbStartTracking->setEnabled(0);
        ui->pbStopTracking->setEnabled(0);
        this->mountMotion.RADriveIsMoving=true;
        maxRASteps=180/g_AllData->getGearData(3)*g_AllData->getGearData(8)*
                g_AllData->getGearData(0)*g_AllData->getGearData(1)*
                g_AllData->getGearData(2); // travel 180° at most

        this->mountMotion.RADriveDirection=-1;
        bwdFactor=this->mountMotion.RASpeedFactor-1; // backward motion means stop at tracking speeds
        futureStepperBehaviourRA =
                QtConcurrent::run(this->StepperDriveRA,
                &QStepperPhidgetsRA::travelForNSteps,maxRASteps,
                                  this->mountMotion.RADriveDirection,
                                  bwdFactor,true);
        while (!futureStepperBehaviourRA.isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        }
    } else {
        this->mountMotion.RADriveIsMoving=false;
        this->StepperDriveRA->stopDrive();
        while (!futureStepperBehaviourRA.isFinished()) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        }
        if (this->mountMotion.RATrackingIsOn == false) {
            this->setControlsForRATravel(true);
        }
        this->startRATracking();
        ui->pbRAPlus->setEnabled(1);
        ui->rbCorrSpeed->setEnabled(true);
        ui->rbMoveSpeed->setEnabled(true);
        ui->pbPGRAMinus->setEnabled(true);
        ui->pbPGRAPlus->setEnabled(true);
        ui->pbPGDecMinus->setEnabled(true);
        ui->pbPGDecPlus->setEnabled(true);
        if (ui->rbMoveSpeed->isChecked()==false) {
            ui->sbMoveSpeed->setEnabled(true);
        }
    }
}

//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
// pulse guide routines
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
void MainWindow::declPGPlus(void) {
    long duration;

    duration = ui->sbPulseGuideDuration->value();
    declinationPulseGuide(duration, 1);
}

//--------------------------------------------------------------
void MainWindow::declPGMinus(void) {
    long duration;

    duration = ui->sbPulseGuideDuration->value();
    declinationPulseGuide(duration, -1);
}
//--------------------------------------------------------------
void MainWindow::declinationPulseGuide(long pulseDurationInMS, short direction) {
    long steps;
    double declSpeed;
    short ldir;

    this->setControlsForDeclTravel(false);
    ui->pbDeclDown->setEnabled(false);
    ui->pbDeclUp->setEnabled(false);
    ui->pbRAMinus->setEnabled(false);
    ui->pbRAPlus->setEnabled(false);
    if (this->mountMotion.DeclDriveIsMoving==true){
        this->mountMotion.DeclDriveIsMoving=false;
        this->StepperDriveDecl->stopDrive();
        while (!futureStepperBehaviourDecl.isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        }
    } // if the decl drive was moving, it is now set to stop
    this->setCorrectionSpeed();
    ui->rbCorrSpeed->setChecked(true); // switch to correction speed
    declSpeed = 0.0041780746*
            (g_AllData->getGearData(4))*
            (g_AllData->getGearData(5))*
            (g_AllData->getGearData(6))*
            (g_AllData->getGearData(8))/(g_AllData->getGearData(7));
    if (direction < 0) {
        ldir = -1;
    } else {
        ldir = 1;
    }
    steps = declSpeed*(pulseDurationInMS/1000.0);
    this->mountMotion.DeclDriveDirection=ldir;
    this->mountMotion.DeclMoveElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
    this->mountMotion.DeclDriveIsMoving=true;
    futureStepperBehaviourDecl =
            QtConcurrent::run(this->StepperDriveDecl,
            &QStepperPhidgetsDecl::travelForNSteps,steps,
                              this->mountMotion.DeclDriveDirection,
                              this->mountMotion.DeclSpeedFactor,0);
    while (!futureStepperBehaviourDecl.isFinished()) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
    }
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

    duration = ui->sbPulseGuideDuration->value();
    raPulseGuide(duration,1);
}

//---------------------------------------------------------------------

void MainWindow::raPGBwd(void) {
    long duration;

    duration = ui->sbPulseGuideDuration->value();
    raPulseGuide(duration,-1);
}
//---------------------------------------------------------------------

void MainWindow::raPulseGuide(long pulseDurationInMS, short direction) {
    long steps;
    double raSpeed,pgFactor;
    QElapsedTimer *localTimer;

    if (this->mountMotion.RATrackingIsOn) {
        this->stopRATracking();
    }
    if (this->mountMotion.RADriveIsMoving==true){
        this->mountMotion.RADriveIsMoving=false;
        this->StepperDriveRA->stopDrive();
        while (!futureStepperBehaviourDecl.isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        }
    }
    this->setControlsForRATravel(false);
    ui->pbStartTracking->setEnabled(0);
    ui->pbStopTracking->setEnabled(0);
    ui->pbRAMinus->setEnabled(0);
    ui->pbRAPlus->setEnabled(0); // if the RA drive was moving, it is now set to stop
    ui->pbDeclDown->setEnabled(0);
    ui->pbDeclUp->setEnabled(0);
    this->setCorrectionSpeed();
    ui->rbCorrSpeed->setChecked(true); // switch to correction speed
    if (direction < 0) {
        direction = -1;
         pgFactor=this->mountMotion.RASpeedFactor-1;
    } else {
        direction = 1;
        pgFactor=this->mountMotion.RASpeedFactor+1;
    }
    if (direction == 1) {
        this->mountMotion.RADriveDirection=direction;
        raSpeed=0.0041780746*
                (g_AllData->getGearData(0))*
                (g_AllData->getGearData(1))*
                (g_AllData->getGearData(2))*
                (g_AllData->getGearData(8))/(g_AllData->getGearData(3));
        steps = direction*pgFactor*raSpeed*(pulseDurationInMS/1000.0);
        this->mountMotion.RAMoveElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
        this->mountMotion.RADriveIsMoving=true;
        this->futureStepperBehaviourRA =
                QtConcurrent::run(this->StepperDriveRA,
                &QStepperPhidgetsRA::travelForNSteps,steps,
                                this->mountMotion.RADriveDirection,pgFactor,false);
        while (!futureStepperBehaviourRA.isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        }
    } else {
        localTimer = new QElapsedTimer();
        localTimer->start();
        while (localTimer->elapsed() < pulseDurationInMS) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
            // that one is easy - just stop the drive for a given amount of time
        }
        delete localTimer;
    }
    this->mountMotion.RADriveIsMoving=false;
    ui->pbRAMinus->setEnabled(1);
    ui->pbRAPlus->setEnabled(1);
    ui->pbDeclDown->setEnabled(1);
    ui->pbDeclUp->setEnabled(1);
    this->setControlsForRATravel(true);
    this->startRATracking();
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
    ui->sbPulseGuideDuration->setEnabled(isEnabled);
    ui->pbPGDecMinus->setEnabled(isEnabled);
    ui->pbPGDecPlus->setEnabled(isEnabled);
    ui->pbPGRAMinus->setEnabled(isEnabled);
    ui->pbPGRAPlus->setEnabled(isEnabled);
    ui->pbSelectGuideStar->setEnabled(isEnabled);
    ui->sbExposureTime->setEnabled(isEnabled);
    ui->tabCCDAcq->setEnabled(isEnabled);
    ui->pbSelectGuideStar->setEnabled(isEnabled);
    ui->hsThreshold->setEnabled(isEnabled);
    ui->hsIContrast->setEnabled(isEnabled);
    ui->hsIBrightness->setEnabled(isEnabled);
    ui->cbMedianFilter->setEnabled(isEnabled);
    ui->rbFOVDbl->setEnabled(isEnabled);
    ui->rbFOVHalf->setEnabled(isEnabled);
    ui->rbFOVStd->setEnabled(isEnabled);
    ui->gearTab->setEnabled(isEnabled);
    ui->LX200Tab->setEnabled(isEnabled);
    ui->catTab->setEnabled(isEnabled);
    ui->ctrlTab->setEnabled(isEnabled);
    ui->hsThreshold->setEnabled(isEnabled);
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
    ui->pbPGDecMinus->setEnabled(isEnabled);
    ui->pbPGDecPlus->setEnabled(isEnabled);
    ui->pbPGRAMinus->setEnabled(isEnabled);
    ui->pbPGRAPlus->setEnabled(isEnabled);
}
//---------------------------------------------------------------------
void MainWindow::setControlsForRATracking(bool isEnabled) {
    ui->leAMaxRA->setEnabled(isEnabled);
    ui->leCurrMaxRA->setEnabled(isEnabled);
    ui->leRAPlanetary->setEnabled(isEnabled);
    ui->leRAGear->setEnabled(isEnabled);
    ui->leRAWorm->setEnabled(isEnabled);
    ui->leRAStepsize->setEnabled(isEnabled);
    ui->leMicrosteps->setEnabled(isEnabled);
    ui->cbIsOnNorthernHemisphere->setEnabled(isEnabled);
}

//---------------------------------------------------------------------

void MainWindow::setControlsForRATravel(bool isEnabled) {
    ui->leAMaxRA->setEnabled(isEnabled);
    ui->leCurrMaxRA->setEnabled(isEnabled);
    ui->leRAPlanetary->setEnabled(isEnabled);
    ui->leRAGear->setEnabled(isEnabled);
    ui->leRAWorm->setEnabled(isEnabled);
    ui->leRAStepsize->setEnabled(isEnabled);
    ui->leMicrosteps->setEnabled(isEnabled);
    ui->cbIsOnNorthernHemisphere->setEnabled(isEnabled);
    ui->pbPGRAMinus->setEnabled(isEnabled);
    ui->pbPGRAPlus->setEnabled(isEnabled);
    ui->listWidgetCatalog->setEnabled(isEnabled);
    ui->listWidgetObject->setEnabled(isEnabled);
    ui->pbSync->setEnabled(isEnabled);
    ui->pbGoTo->setEnabled(isEnabled);
    ui->pbStop2->setEnabled(true);
}

//---------------------------------------------------------------------
void MainWindow::setControlsForDeclTravel(bool isEnabled) {
    ui->leAMaxDecl->setEnabled(isEnabled);
    ui->leCurrMaxDecl->setEnabled(isEnabled);
    ui->rbCorrSpeed->setEnabled(isEnabled);
    ui->rbMoveSpeed->setEnabled(isEnabled);
    ui->sbMoveSpeed->setEnabled(isEnabled);
    ui->leDeclPlanetary->setEnabled(isEnabled);
    ui->leDeclGear->setEnabled(isEnabled);
    ui->leDeclWorm->setEnabled(isEnabled);
    ui->leDeclStepSize->setEnabled(isEnabled);
    ui->leMicrosteps->setEnabled(isEnabled);
    ui->pbPGDecMinus->setEnabled(isEnabled);
    ui->pbPGDecPlus->setEnabled(isEnabled);
    ui->listWidgetCatalog->setEnabled(isEnabled);
    ui->listWidgetObject->setEnabled(isEnabled);
    ui->pbSync->setEnabled(isEnabled);
    ui->pbGoTo->setEnabled(isEnabled);
    ui->pbStop2->setEnabled(true);
}

//---------------------------------------------------------------------
void MainWindow::setControlsForAutoguiderCalibration(bool isEnabled) {
    ui->ctrlTab->setEnabled(isEnabled);
    ui->catTab->setEnabled(isEnabled);
    ui->LX200Tab->setEnabled(isEnabled);
    ui->camTab->setEnabled(isEnabled);
    ui->tabCCDAcq->setEnabled(isEnabled);
    ui->tabGuideParams->setEnabled(isEnabled);
    ui->tabImageProc->setEnabled(isEnabled);
    ui->ccdTab->setEnabled(isEnabled);
    ui->gearTab->setEnabled(isEnabled);
}

//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
// routines for handling the .tsc catalogs
//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
void MainWindow::catalogChosen(QListWidgetItem* catalogName)
{
    QString *catalogPath;
    long counterForObjects, maxObj;
    std::string objectName;

    ui->listWidgetCatalog->blockSignals(true);
    if (this->objCatalog != NULL) {
        delete this->objCatalog;
    }
    if (this->MountWasSynced == true) {
        ui->pbGoTo->setEnabled(true);
    }
    ui->pbSync->setEnabled(false);
    catalogPath = new QString(catalogName->text());
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
}
//------------------------------------------------------------------

void MainWindow::catalogObjectChosen(void)
{
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
            this->decl=epDecl+deltaDecl*((double)(ui->sbEpoch->value()-ui->lcdCatEpoch->value()));
            qDebug() << "catalog ra vs. corr. RA:" << epRA << this->ra;
            qDebug() << "catalog decl vs. corr. decl:" << epDecl << this->decl;
        }

        lestr = QString::number(this->ra, 'g', 8);
        ui->lineEditRA->setText(lestr);
        lestr = QString::number(this->decl, 'g', 8);
        ui->lineEditDecl->setText(lestr);
        ui->pbSync->setEnabled(true);
    }
}

//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
// store drive and gear data to g_AllData and preference file
//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------
void MainWindow::storeGearData(void) {
    float pgra,ogra,wormra,ssra,pgdec,ogdec,wormdec,ssdec,microsteps,vra,vdecl;
    QString *leEntry,leSpeeds;

    leEntry = new QString(ui->leRAPlanetary->text());
    pgra=leEntry->toFloat();
    leEntry->clear();
    leEntry->append(ui->leRAGear->text());
    ogra=leEntry->toFloat();
    leEntry->clear();
    leEntry->append(ui->leRAWorm->text());
    wormra=leEntry->toFloat();
    leEntry->clear();
    leEntry->append(ui->leRAStepsize->text());
    ssra=leEntry->toFloat();
    leEntry->clear();
    leEntry->append(ui->leDeclPlanetary->text());
    pgdec=leEntry->toFloat();
    leEntry->clear();
    leEntry->append(ui->leDeclGear->text());
    ogdec=leEntry->toFloat();
    leEntry->clear();
    leEntry->append(ui->leDeclWorm->text());
    wormdec=leEntry->toFloat();
    leEntry->clear();
    leEntry->append(ui->leDeclStepSize->text());
    ssdec=leEntry->toFloat();
    leEntry->clear();
    leEntry->append(ui->leMicrosteps->text());
    microsteps=leEntry->toFloat();
    g_AllData->setGearData(pgra,ogra,wormra,ssra,pgdec,ogdec,wormdec,ssdec,microsteps);
    // store all gear data in global struct
    g_AllData->storeGlobalData();

    if (this->StepperDriveRA->getStopped() == false) {
        this->stopRATracking();
        StepperDriveRA->changeSpeedForGearChange();
        StepperDriveDecl->changeSpeedForGearChange();
        this->startRATracking();
    } else {
        StepperDriveRA->changeSpeedForGearChange();
        StepperDriveDecl->changeSpeedForGearChange();
    }
    vra=StepperDriveRA->getKinetics(3);
    leSpeeds= QString::number(vra, 'g', 2);
    ui->leVMaxRA->setText(leSpeeds);
    vdecl=StepperDriveDecl->getKinetics(3);
    leSpeeds= QString::number(vdecl, 'g', 2);
    ui->leVMaxDecl->setText(leSpeeds);
    delete leEntry;
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
