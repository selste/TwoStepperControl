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
MainWindow::MainWindow(QWidget *parent):QMainWindow(parent), ui(new Ui::MainWindow)
{
    int serNo; // serial number of the phidgets boards
    double val,draccRA, draccDecl, drcurrRA, drcurrDecl; // local values on drive acceleration and so on...
    QStepperPhidgetsRA *dummyDrive;
    QDir *catalogDir;
    QFileInfoList catFiles;
    QFileInfo catFileInfo;
    QStringList filter;
    QString *catfName; // a bunch of local variables to read the catalogues

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

    this->mountMotion.RATrackingIsOn=false;
    this->mountMotion.RADriveIsMoving =false;
    this->mountMotion.DeclDriveIsMoving = false;
    this->mountMotion.GoToIsActiveInRA = false;
    this->mountMotion.GoToIsActiveInDecl = false; // setting a few flags on drive states
    this->mountMotion.emergencyStopTriggered = false;
    this->lx200IsOn = false;
    this->MountWasSynced = false;
    this->mountMotion.DeclDriveDirection = 1;
    this->mountMotion.RADriveDirection = 1; // 1 for forward, -1 for backward
    this->mountMotion.RASpeedFactor=1;
    this->mountMotion.DeclSpeedFactor=1; // speeds are multiples of sidereal compensation
    ui->rbCorrSpeed->setChecked(true); // activate radiobutton for correction speed ... this is sidereal speed

    g_AllData->setDriveParams(0,0,this->StepperDriveRA->getKinetics(3));
    g_AllData->setDriveParams(1,0,this->StepperDriveDecl->getKinetics(3)); // velocity limit - this is set to sidereal speed for both declination and right ascension in the constructor ...
    this->StepperDriveRA->setStepperParams((g_AllData->getDriveParams(0,1)),1); // acceleration in RA
    this->StepperDriveRA->setStepperParams((g_AllData->getDriveParams(0,2)),3); // motor current in RA
    this->StepperDriveDecl->setStepperParams((g_AllData->getDriveParams(1,1)),1); // acceleration in Decl
    this->StepperDriveDecl->setStepperParams((g_AllData->getDriveParams(1,2)),3); // motor current in Decl
        // now setting all the parameters in the "Drive"-tab
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
    // take settings from the pref-file, except for the stepper speed, which is
    // calculated from  gear parameters

    camera_client = new alccd5_client(); // install a camera client for guiding via INDI
    connect(this->camera_client,SIGNAL(imageAvailable()),this,SLOT(displayGuideCamImage()),Qt::QueuedConnection);
    connect(this->camera_client,SIGNAL(messageFromINDIAvailable()),this,SLOT(handleServerMessage()),Qt::QueuedConnection);

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

    connect(timer, SIGNAL(timeout()), this, SLOT(updateReadings())); // this is the event queue
    connect(ui->pbExit,SIGNAL(clicked()), this, SLOT(shutDownProgram())); // this kills teh program, including killing the drives
    connect(ui->pbConnectToServer,SIGNAL(clicked()),this, SLOT(setINDISAddrAndPort())); // connects to the INDI server at the given address ...
    connect(ui->pbExpose, SIGNAL(clicked()), this, SLOT(takeSingleCamShot())); // take one shot from the ccd-camera
    connect(ui->listWidgetCatalog,SIGNAL(itemClicked(QListWidgetItem*)),this,SLOT(catalogChosen(QListWidgetItem*)));
    connect(ui->listWidgetObject,SIGNAL(itemClicked(QListWidgetItem*)),this,SLOT(catalogObjectChosen(void))); // catalog selection
    connect(this->camView,SIGNAL(currentViewStatusSignal(QPointF)),this->camView,SLOT(currentViewStatusSlot(QPointF))); // position the crosshair in the camera view ...
    connect(ui->pbSync, SIGNAL(clicked()), this, SLOT(syncMount())); // reset the current position and timer, and set the global mount position to the actual coordinates
    connect(ui->pbStoreGears, SIGNAL(clicked()), this, SLOT(storeGearData()));
    connect(ui->pbStartTracking, SIGNAL(clicked()),this,SLOT(startRATracking()));
    connect(ui->pbStopTracking, SIGNAL(clicked()),this,SLOT(stopRATracking()));
    connect(ui->pbDeclUp, SIGNAL(clicked()),this,SLOT(declinationMoveHandboxUp()));
    connect(ui->pbDeclDown, SIGNAL(clicked()),this,SLOT(declinationMoveHandboxDown()));
    connect(ui->pbRAPlus, SIGNAL(clicked()),this,SLOT(RAMoveHandboxFwd()));
    connect(ui->pbRAMinus, SIGNAL(clicked()),this,SLOT(RAMoveHandboxBwd()));
    connect(ui->leAMaxRA, SIGNAL(textChanged(QString)), this, SLOT(setMaxStepperAccRA()));
    connect(ui->leCurrMaxRA, SIGNAL(textChanged(QString)), this, SLOT(setMaxStepperCurrentRA()));
    connect(ui->leAMaxDecl, SIGNAL(textChanged(QString)), this, SLOT(setMaxStepperAccDecl()));
    connect(ui->leCurrMaxDecl, SIGNAL(textChanged(QString)), this, SLOT(setMaxStepperCurrentDecl()));
    connect(ui->pbStoreDrive, SIGNAL(clicked()), this, SLOT(storeDriveData()));
    connect(ui->rbCorrSpeed,SIGNAL(released()), this, SLOT(setCorrectionSpeed()));
    connect(ui->rbMoveSpeed,SIGNAL(released()), this, SLOT(setMoveSpeed()));
    connect(ui->pbGoTo, SIGNAL(clicked()),this, SLOT(startGoToObject()));
    connect(ui->sbMoveSpeed, SIGNAL(valueChanged(int)),this,SLOT(changeMoveSpeed()));
    connect(ui->cbIsOnNorthernHemisphere, SIGNAL(stateChanged(int)), this, SLOT(invertRADirection()));
    connect(ui->pbLX200Active, SIGNAL(clicked()), this, SLOT(switchToLX200()));
    connect(ui->pbGetCCDParams, SIGNAL(clicked()), this, SLOT(getCCDParameters()));
    connect(ui->pbStoreCCDParams, SIGNAL(clicked()), this, SLOT(storeCCDData()));
    connect(ui->pbStartINDIServer, SIGNAL(clicked()), this, SLOT(deployINDICommand()));
    connect(ui->pbStop1, SIGNAL(clicked()), this, SLOT(emergencyStop()));
    connect(ui->pbStop2, SIGNAL(clicked()), this, SLOT(emergencyStop()));
    connect(ui->pbStop3, SIGNAL(clicked()), this, SLOT(emergencyStop()));
    connect(ui->pbStop4, SIGNAL(clicked()), this, SLOT(emergencyStop()));
    connect(ui->pbPGDecPlus, SIGNAL(clicked()), this, SLOT(declPGPlus()));
    connect(ui->pbPGDecMinus, SIGNAL(clicked()), this, SLOT(declPGMinus()));
    connect(ui->pbPGRAPlus, SIGNAL(clicked()), this, SLOT(raPGFwd()));
    connect(ui->pbPGRAMinus, SIGNAL(clicked()), this, SLOT(raPGBwd()));
    connect(ui->pbClearLXLog, SIGNAL(clicked()), this, SLOT(clearLXLog()));

    RAdriveDirectionForNorthernHemisphere = 1; //switch this for the southern hemisphere to -1 ... RA is inverted
    g_AllData->storeGlobalData();
    g_AllData->setSyncPosition(0.0, 0.0); // deploy a fake sync to the mount so that the microtimer starts ...

    this->lx200port= new lx200_communication();
    if (this->lx200port->getPortState() == 1) {
        ui->cbRS232Open->setChecked(true);
    }
    this->LXSetNumberFormatToSimple();
    connect(this->lx200port,SIGNAL(RS232moveEast()), this, SLOT(LXmoveEast()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232moveWest()), this, SLOT(LXmoveWest()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232moveNorth()), this, SLOT(LXmoveNorth()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232moveSouth()), this, SLOT(LXmoveSouth()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232stopMoveEast()), this, SLOT(LXstopMoveEast()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232stopMoveWest()), this, SLOT(LXstopMoveWest()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232stopMoveNorth()), this, SLOT(LXstopMoveNorth()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232stopMoveSouth()), this, SLOT(LXstopMoveSouth()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232stopMotion()), this, SLOT(LXstopMotion()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232guideSpeed()), this, SLOT(LXslowSpeed()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232centerSpeed()), this, SLOT(LXslowSpeed()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232findSpeed()), this, SLOT(LXhiSpeed()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232gotoSpeed()), this, SLOT(LXhiSpeed()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232sync()),this,SLOT(LXsyncMount()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232slew()),this,SLOT(LXslewMount()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232CommandReceived()),this, SLOT(logLX200IncomingCmds()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232RASent()),this, SLOT(logLX200OutgoingCmdsRA()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232DeclSent()),this, SLOT(logLX200OutgoingCmdsDecl()),Qt::QueuedConnection);
    connect(this->lx200port,SIGNAL(RS232CommandSent()),this, SLOT(logLX200OutgoingCmds()),Qt::QueuedConnection);
    connect(ui->cbLXSimpleNumbers, SIGNAL(released()),this, SLOT(LXSetNumberFormatToSimple()));
    this->StepperDriveRA->stopDrive();
    this->StepperDriveDecl->stopDrive(); // just to kill all jobs that may lurk in the muproc ...
}
//------------------------------------------------------------------
MainWindow::~MainWindow()
{
    delete StepperDriveRA;
    delete StepperDriveDecl;
    delete timer;
    delete textEntry;
    delete lx200port;
    delete ui;
    exit(0);
}
//------------------------------------------------------------------
void MainWindow::updateReadings()
{
    qint64 topicalTime,charsReadFromRS232;
    double relativeTravelRA, relativeTravelDecl,totalGearRatio;

    if (this->lx200IsOn) {
        if (lx200port->getPortState() == 1) {
            charsReadFromRS232 = lx200port->getDataFromSerialPort();
        }
    }
    if (this->mountMotion.RATrackingIsOn == true) {
        topicalTime = g_AllData->getTimeSinceLastSync() - this->mountMotion.RAtrackingElapsedTimeInMS;
        this->mountMotion.RAtrackingElapsedTimeInMS+=topicalTime;
        totalGearRatio = g_AllData->getGearData(0)*g_AllData->getGearData(1)*
        g_AllData->getGearData(2);
        relativeTravelRA= this->StepperDriveRA->getKinetics(3)*topicalTime*g_AllData->getGearData(3)/
                (1000.0*g_AllData->getGearData(8)*totalGearRatio);
        g_AllData->incrementActualScopePosition(relativeTravelRA, 0.0);
    }
    if (this->mountMotion.RADriveIsMoving == true) {
        topicalTime = g_AllData->getTimeSinceLastSync() - this->mountMotion.RAMoveElapsedTimeInMS;
        this->mountMotion.RAMoveElapsedTimeInMS+=topicalTime;
        totalGearRatio = g_AllData->getGearData(0)*g_AllData->getGearData(1)*
        g_AllData->getGearData(2);
        relativeTravelRA=this->mountMotion.RADriveDirection*
                this->StepperDriveRA->getKinetics(3)*topicalTime*g_AllData->getGearData(3)/
                (1000.0*g_AllData->getGearData(8)*totalGearRatio);
        g_AllData->incrementActualScopePosition(relativeTravelRA, 0.0);
    }

    if (this->mountMotion.DeclDriveIsMoving == true) {
        topicalTime = g_AllData->getTimeSinceLastSync() - this->mountMotion.DeclMoveElapsedTimeInMS;
        this->mountMotion.DeclMoveElapsedTimeInMS+=topicalTime;
        totalGearRatio = g_AllData->getGearData(4)*g_AllData->getGearData(5)*
        g_AllData->getGearData(6);
        relativeTravelDecl= this->mountMotion.DeclDriveDirection*
                this->StepperDriveDecl->getKinetics(3)*topicalTime*g_AllData->getGearData(7)/
                (1000.0*g_AllData->getGearData(8)*totalGearRatio);
        g_AllData->incrementActualScopePosition(0.0, relativeTravelDecl);
    }

    if ((this->mountMotion.GoToIsActiveInRA==true) || (this->mountMotion.GoToIsActiveInDecl==true)) {
        ui->lcdGotoTime->display(round((this->gotoETA-this->elapsedGoToTime->elapsed())*0.001));
        if (this->mountMotion.GoToIsActiveInRA==true) {
            topicalTime = g_AllData->getTimeSinceLastSync() - this->mountMotion.RAGoToElapsedTimeInMS;
            this->mountMotion.RAGoToElapsedTimeInMS+=topicalTime;
            totalGearRatio = g_AllData->getGearData(0)*g_AllData->getGearData(1)*
            g_AllData->getGearData(2);
            relativeTravelRA=this->mountMotion.RADriveDirection*
                    this->approximateGOTOSpeedRA*topicalTime*g_AllData->getGearData(3)/
                    (1000.0*g_AllData->getGearData(8)*totalGearRatio);
            g_AllData->incrementActualScopePosition(relativeTravelRA, 0.0);
        }
        if (this->mountMotion.GoToIsActiveInDecl==true) {
            topicalTime = g_AllData->getTimeSinceLastSync() - this->mountMotion.DeclGoToElapsedTimeInMS;
            this->mountMotion.DeclGoToElapsedTimeInMS+=topicalTime;
            totalGearRatio = g_AllData->getGearData(4)*g_AllData->getGearData(5)*
            g_AllData->getGearData(6);
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
    }
    ui->leHourAngle->setText(textEntry->number(g_AllData->getActualScopePosition(0),'f',5));
    ui->leDecl->setText(textEntry->number(g_AllData->getActualScopePosition(1),'f',5));
}
//------------------------------------------------------------------
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
void MainWindow::shutDownProgram() {
    ui->cbContinuous->setChecked(false);
    sleep(ui->sbExposureTime->value());
    camera_client->sayGoodbyeToINDIServer();
    this->StepperDriveRA->stopDrive();
    delete StepperDriveRA;
    this->StepperDriveDecl->stopDrive();
    delete StepperDriveDecl;
    exit(0);
}
//------------------------------------------------------------------
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
void MainWindow::setMaxStepperCurrentDecl(void)
{
    double val;
    QString *leEntry;

    leEntry = new QString(ui->leCurrMaxDecl->text());
    val = (double)(leEntry->toFloat());
    this->StepperDriveDecl->setStepperParams(val, 3);
    g_AllData->setDriveParams(1,2,val);
    delete leEntry;
}
//------------------------------------------------------------------
void MainWindow::setINDISAddrAndPort(void)
{
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
    }
}
//------------------------------------------------------------------
void MainWindow::takeSingleCamShot(void)
{
   int exptime;

   exptime = (ui->sbExposureTime->value());
   camera_client->takeExposure(exptime);
}

//------------------------------------------------------------------

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

void MainWindow::clearLXLog(void) {
    ui->teLX200Data->clear();
}

//------------------------------------------------------------------

void MainWindow::deployINDICommand(void) {
    int retval;

    if (ui->rbQHYINDI->isChecked()== true) {
        retval = system("indiserver -v -m 100 indi_qhy_ccd &");
    }
    ui->pbStartINDIServer->setEnabled(false);
}

//------------------------------------------------------------------

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

void MainWindow::updateCameraImage(void)
{
    camImg=camera_client->getScaledPixmapFromCamera();
    this->camView->addBgImage(*camImg);
}
//------------------------------------------------------------------
void MainWindow::displayGuideCamImage(void) {

    if (g_AllData->getINDIState() == true) {
        this->updateCameraImage();
        if (ui->cbContinuous->isChecked()) {
          this->takeSingleCamShot();
        }
    }
}

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
            raRadians=epDecl/180.0*3.141592653589793;
            deltaRA = meeusM+meeusN*sin(raRadians)*tan(raRadians);
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
void MainWindow::syncMount(void)
{
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
    g_AllData->setSyncPosition(this->ra, this->decl);
    // convey right ascension and declination to the global parameters;
    // a microtimer starts ...
    this->startRATracking();
    ui->pbGoTo->setEnabled(true);
    this->MountWasSynced = true;
}
//------------------------------------------------------------------

void MainWindow::LXsyncMount(void)
{
    QString lestr;

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

//------------------------------------------------------------------
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
void MainWindow::storeGearData(void)
{
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
//--------------------------------------------------------------
void MainWindow::declinationMoveHandboxUp(void)
{
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
                                  this->mountMotion.DeclSpeedFactor);
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
        direction = -1;
    } else {
        direction = 1;
    }
    steps = direction*declSpeed*(pulseDurationInMS/1000.0);
    this->mountMotion.DeclDriveDirection=direction;
    this->mountMotion.DeclMoveElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
    this->mountMotion.DeclDriveIsMoving=true;
    futureStepperBehaviourDecl =
            QtConcurrent::run(this->StepperDriveDecl,
            &QStepperPhidgetsDecl::travelForNSteps,steps,
                              this->mountMotion.DeclDriveDirection,
                              this->mountMotion.DeclSpeedFactor);
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

//--------------------------------------------------------------

void MainWindow::declinationMoveHandboxDown(void)
{
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
                                  this->mountMotion.DeclSpeedFactor);
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

void MainWindow::RAMoveHandboxFwd(void)
{
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
                                  fwdFactor);
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
                                  bwdFactor);
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
                                this->mountMotion.RADriveDirection,pgFactor);
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
//---------------------------------------------------------------------

void MainWindow::LXmoveEast(void) {
    if ((mountMotion.GoToIsActiveInRA==false) ||
            (mountMotion.GoToIsActiveInDecl==false)) {
        if (this->mountMotion.RADriveIsMoving == true) {
            return;
        } else {
            this->RAMoveHandboxBwd();
        }
    }
}

//---------------------------------------------------------------------

void MainWindow::LXmoveWest(void) {

    if ((mountMotion.GoToIsActiveInRA==false) ||
            (mountMotion.GoToIsActiveInDecl==false)) {
        if (this->mountMotion.RADriveIsMoving == true) {
            return;
        } else {
            this->RAMoveHandboxFwd();
        }
    }
}

//---------------------------------------------------------------------

void MainWindow::LXmoveNorth(void) {
    if ((mountMotion.GoToIsActiveInRA==false) ||
            (mountMotion.GoToIsActiveInDecl==false)) {
        if (this->mountMotion.DeclDriveIsMoving == true) {
            return;
        } else {
            this->declinationMoveHandboxUp();
        }
    }
}

//---------------------------------------------------------------------

void MainWindow::LXmoveSouth(void) {

    if ((mountMotion.GoToIsActiveInRA==false) ||
            (mountMotion.GoToIsActiveInDecl==false)) {
        if (this->mountMotion.DeclDriveIsMoving == true) {
            return;
        } else {
            this->declinationMoveHandboxDown();
        }
    }
}

//---------------------------------------------------------------------

void MainWindow::LXstopMoveEast(void) {
    if (((mountMotion.GoToIsActiveInRA==false) ||
            (mountMotion.GoToIsActiveInDecl==false)) &&
            (mountMotion.RADriveIsMoving == true))  {
        this->RAMoveHandboxBwd();
    }
}

//---------------------------------------------------------------------

void MainWindow::LXstopMoveWest(void) {
    if (((mountMotion.GoToIsActiveInRA==false) ||
            (mountMotion.GoToIsActiveInDecl==false)) &&
            (mountMotion.RADriveIsMoving == true))  {
        this->RAMoveHandboxFwd();
    }
}

//---------------------------------------------------------------------

void MainWindow::LXstopMoveNorth(void) {
    if (((mountMotion.GoToIsActiveInRA==false) ||
            (mountMotion.GoToIsActiveInDecl==false)) &&
            (mountMotion.DeclDriveIsMoving == true))  {
        this->declinationMoveHandboxUp();
    }
}

//---------------------------------------------------------------------

void MainWindow::LXstopMoveSouth(void) {
    if (((mountMotion.GoToIsActiveInRA==false) ||
            (mountMotion.GoToIsActiveInDecl==false)) &&
            (mountMotion.DeclDriveIsMoving == true))  {
        this->declinationMoveHandboxDown();
    }
}

//---------------------------------------------------------------------

void MainWindow::setControlsForRATracking(bool isEnabled)
{
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
   
void MainWindow::setControlsForRATravel(bool isEnabled)
{
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
    ui->catTab->setEnabled(isEnabled);
}

//---------------------------------------------------------------------

void MainWindow::setControlsForDeclTravel(bool isEnabled)
{
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
    ui->catTab->setEnabled(isEnabled);
}

//---------------------------------------------------------------------

void MainWindow::setCorrectionSpeed(void)
{
    this->mountMotion.RASpeedFactor = 1;
    this->mountMotion.DeclSpeedFactor = 1;
    ui->sbMoveSpeed->setEnabled(true);
}

//---------------------------------------------------------------------

void MainWindow::LXslowSpeed(void)
{
    ui->rbCorrSpeed->setChecked(true);
    this->setCorrectionSpeed();

}

//---------------------------------------------------------------------

void MainWindow::LXhiSpeed(void)
{
    ui->rbMoveSpeed->setChecked(true);
    this->setMoveSpeed();
}

//---------------------------------------------------------------------

void MainWindow::LXSetNumberFormatToSimple(void) {
    if (ui->cbLXSimpleNumbers->isChecked() == true) {
        this->lx200port->setNumberFormat(true);
    } else {
        this->lx200port->setNumberFormat(false);
    }
}

//---------------------------------------------------------------------

void MainWindow::setMoveSpeed(void)
{
    this->mountMotion.RASpeedFactor = ui->sbMoveSpeed->value();
    this->mountMotion.DeclSpeedFactor = ui->sbMoveSpeed->value();
    ui->sbMoveSpeed->setEnabled(false);
}

//---------------------------------------------------------------------

void MainWindow::storeDriveData(void)
{
    g_AllData->storeGlobalData();
    this->StepperDriveRA->setStepperParams(g_AllData->getDriveParams(0,1),1);//acc
    this->StepperDriveRA->setStepperParams(g_AllData->getDriveParams(0,2),3);//current
    this->StepperDriveDecl->setStepperParams(g_AllData->getDriveParams(1,1),1);//acc
    this->StepperDriveDecl->setStepperParams(g_AllData->getDriveParams(1,2),3);//current
}

//---------------------------------------------------------------------

void MainWindow::changeMoveSpeed(void) {
    this->mountMotion.RASpeedFactor = ui->sbMoveSpeed->value();
    this->mountMotion.DeclSpeedFactor = ui->sbMoveSpeed->value();
    ui->rbMoveSpeed->setChecked(true);
}

//---------------------------------------------------------------------

void MainWindow::invertRADirection(void) {
    if (ui->cbIsOnNorthernHemisphere->isChecked() == true) {
        this->RAdriveDirectionForNorthernHemisphere = 1;
    } else {
        this->RAdriveDirectionForNorthernHemisphere = -1;
    }
    this->StepperDriveRA->setRADirection(this->RAdriveDirectionForNorthernHemisphere);
}

//---------------------------------------------------------------------

void MainWindow::setControlsForGoto(bool isEnabled)
{
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

//---------------------------------------------------------------------

void MainWindow::LXstopMotion(void) {
    this->emergencyStop();
}

//---------------------------------------------------------------------
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
    }     // terminate all current motions ...
}

//---------------------------------------------------------------------

void MainWindow::LXslewMount(void) {
    QString lestr;

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
            qDebug() << "LX Slew to" << this->ra << "and" << this->decl;
            this->startGoToObject();
        } else {
            qDebug() << "Slew impossible - mount not synced";
        }
    }
}

//---------------------------------------------------------------------
void MainWindow::startGoToObject(void) {
    double travelRA, travelDecl, speedFactorRA, speedFactorDecl,TRamp, SRamp,
            SAtFullSpeed, TAtFullSpeed, earthTravelDuringGOTOinMSteps,
            convertDegreesToMicrostepsDecl,convertDegreesToMicrostepsRA;
    float targetRA, targetDecl;
    qint64 timestampGOTOStarted, timeDifference, timeTaken;
    qint64 timeEstimatedInRAInMS = 0;
    qint64 timeEstimatedInDeclInMS = 0;
    long int RASteps, DeclSteps,corrsteps;
    int timeForProcessingEventQueue = 100;
    bool RAtakesLonger, shortSlew;

    ui->pbGoTo->setEnabled(false);
    this->terminateAllMotion();
    this->setControlsForGoto(false);
    ui->pbStartTracking->setEnabled(false);
    shortSlew=false;
    timeDifference=0;
    // determine the travel to be taken based on steps, aceleration and end velocity
    travelRA=((g_AllData->getActualScopePosition(0))+0.0041780746*g_AllData->getTimeSinceLastSync()/1000.0)-this->ra;
    travelDecl=this->decl-g_AllData->getActualScopePosition(1);
    targetRA = this->ra;
    targetDecl = this->decl;
    if (travelRA < 0) {
        this->mountMotion.RADriveDirection = -1;
    } else {
        this->mountMotion.RADriveDirection = 1;
    }
    if (travelDecl < 0) {
        this->mountMotion.DeclDriveDirection = -1;
    } else {
        this->mountMotion.DeclDriveDirection = 1;
    }
    speedFactorDecl=ui->sbGoToSpeed->value();
    speedFactorRA=ui->sbGoToSpeed->value();
    convertDegreesToMicrostepsDecl=1.0/g_AllData->getGearData(7)*g_AllData->getGearData(8)*
            g_AllData->getGearData(4)*g_AllData->getGearData(5)*g_AllData->getGearData(6);
    DeclSteps=round(fabs(travelDecl)*convertDegreesToMicrostepsDecl);
    convertDegreesToMicrostepsRA=1.0/g_AllData->getGearData(3)*g_AllData->getGearData(8)*
            g_AllData->getGearData(0)*g_AllData->getGearData(1)*g_AllData->getGearData(2);
    RASteps=round(fabs(travelRA)*convertDegreesToMicrostepsRA);

    TRamp = (this->StepperDriveDecl->getKinetics(3)*(speedFactorDecl))/this->StepperDriveDecl->getKinetics(2);// time needed until drive reaches full speed - vel/acc ...
    SRamp = 0.5*this->StepperDriveDecl->getKinetics(2)*TRamp*TRamp; // travel in microsteps until full speed is reached
    SAtFullSpeed = DeclSteps-2.0*SRamp;
    if (SAtFullSpeed < 0) {
        TAtFullSpeed=sqrt(DeclSteps/this->StepperDriveDecl->getKinetics(2));// if the travel is so short that full speed cannot be reached: consider a ramp that stops at the end of travel
        timeEstimatedInDeclInMS = (TAtFullSpeed)*1000+timeForProcessingEventQueue;
    } else {
        TAtFullSpeed = SAtFullSpeed/(this->StepperDriveDecl->getKinetics(3)*speedFactorDecl);
        timeEstimatedInDeclInMS = (TAtFullSpeed+2.0*TRamp)*1000+timeForProcessingEventQueue;// time in microseconds estimated for Declination-Travel
    }

    // Now repeat that for the RA drive
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
            convertDegreesToMicrostepsRA; // determine the addition travel in sideral time

    if (this->mountMotion.RADriveDirection == 1) {
        RASteps=RASteps+earthTravelDuringGOTOinMSteps;
    } else {
        RASteps=RASteps-earthTravelDuringGOTOinMSteps;
    }
    timeEstimatedInRAInMS = RASteps/((double)this->StepperDriveRA->getKinetics(3)*(speedFactorRA))*1000;

    if (timeEstimatedInDeclInMS > timeEstimatedInRAInMS) {
        gotoETA = timeEstimatedInDeclInMS;
        RAtakesLonger=false;
    } else {
        gotoETA = timeEstimatedInRAInMS;
        RAtakesLonger=true;
    }
    if ((timeEstimatedInDeclInMS < 5000) || (timeEstimatedInRAInMS < 5000)) {
        gotoETA = timeEstimatedInDeclInMS+timeEstimatedInRAInMS;
        shortSlew=true;
    }
    this->approximateGOTOSpeedRA=RASteps/(timeEstimatedInRAInMS/1000.0);
    this->approximateGOTOSpeedDecl=DeclSteps/(timeEstimatedInDeclInMS/1000.0);
    ui->lcdGotoTime->display(round(gotoETA/1000.0));
    // determined the estimated duration of the GoTo - Process
    QCoreApplication::processEvents(QEventLoop::AllEvents, timeForProcessingEventQueue);






 // let the games begin...
    bool RARideIsDone = false;
    elapsedGoToTime->start();

    if (shortSlew == true) {
        futureStepperBehaviourDecl_GOTO =QtConcurrent::run(this->StepperDriveDecl,&QStepperPhidgetsDecl::travelForNSteps,DeclSteps,this->mountMotion.DeclDriveDirection,speedFactorDecl);
        while (!futureStepperBehaviourDecl.isStarted()) {
        }
        this->mountMotion.GoToIsActiveInDecl=true;
        timestampGOTOStarted = g_AllData->getTimeSinceLastSync();
        this->mountMotion.DeclGoToElapsedTimeInMS=g_AllData->getTimeSinceLastSync();
        while (!futureStepperBehaviourDecl_GOTO.isFinished()) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, timeForProcessingEventQueue);
            if (this->mountMotion.emergencyStopTriggered==true) {
                this->mountMotion.emergencyStopTriggered=false;
                return;
            }
        }
        this->mountMotion.GoToIsActiveInDecl=false;

        futureStepperBehaviourRA_GOTO =QtConcurrent::run(this->StepperDriveRA,&QStepperPhidgetsRA::travelForNSteps,RASteps,this->mountMotion.RADriveDirection,speedFactorRA);
        while (!futureStepperBehaviourRA.isStarted()) {
        }
        this->mountMotion.GoToIsActiveInRA=true;
        this->mountMotion.RAGoToElapsedTimeInMS=g_AllData->getTimeSinceLastSync();
        while (!futureStepperBehaviourRA_GOTO.isFinished()) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, timeForProcessingEventQueue);
            if (this->mountMotion.emergencyStopTriggered==true) {
                this->mountMotion.emergencyStopTriggered=false;
                return;
            }
        }
        timeTaken = g_AllData->getTimeSinceLastSync()-timestampGOTOStarted;
        timeDifference = timeTaken-timeEstimatedInRAInMS;
        this->startRATracking();
        ui->pbStopTracking->setDisabled(true);
        this->mountMotion.GoToIsActiveInRA=false;
    } else { // carry out the slews parallel
        futureStepperBehaviourRA_GOTO =QtConcurrent::run(this->StepperDriveRA,&QStepperPhidgetsRA::travelForNSteps,RASteps,this->mountMotion.RADriveDirection,speedFactorRA);
        while (!futureStepperBehaviourRA.isStarted()) {
        }
        this->mountMotion.GoToIsActiveInRA=true;
        this->mountMotion.RAGoToElapsedTimeInMS=g_AllData->getTimeSinceLastSync();

        timestampGOTOStarted = g_AllData->getTimeSinceLastSync();
        futureStepperBehaviourDecl_GOTO =QtConcurrent::run(this->StepperDriveDecl,&QStepperPhidgetsDecl::travelForNSteps,DeclSteps,this->mountMotion.DeclDriveDirection,speedFactorDecl);
        while (!futureStepperBehaviourDecl.isStarted()) {
        }
        this->mountMotion.GoToIsActiveInDecl=true;
        this->mountMotion.DeclGoToElapsedTimeInMS=g_AllData->getTimeSinceLastSync();

        if (RAtakesLonger == true) {
            while (!futureStepperBehaviourRA_GOTO.isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, timeForProcessingEventQueue);
                if (futureStepperBehaviourDecl_GOTO.isFinished()) {
                    this->mountMotion.GoToIsActiveInDecl=false;
                }
                if (this->mountMotion.emergencyStopTriggered==true) {
                    this->mountMotion.emergencyStopTriggered=false;
                    return;
                }
            }

            timeTaken = g_AllData->getTimeSinceLastSync()-timestampGOTOStarted;
            timeDifference = timeTaken-timeEstimatedInRAInMS;
            this->startRATracking();
            ui->pbStopTracking->setDisabled(true);
            this->mountMotion.GoToIsActiveInRA=false;
        } else {
            while (!futureStepperBehaviourDecl_GOTO.isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, timeForProcessingEventQueue);
                if (futureStepperBehaviourRA_GOTO.isFinished()) {
                    this->mountMotion.GoToIsActiveInRA=false;
                }
                if (futureStepperBehaviourRA_GOTO.isFinished()) {
                    if (this->mountMotion.emergencyStopTriggered==true) {
                        this->mountMotion.emergencyStopTriggered=false;
                        return;
                    }
                    if (RARideIsDone==false) {
                        RARideIsDone=true;
                        timeTaken = g_AllData->getTimeSinceLastSync()-timestampGOTOStarted;
                        timeDifference = timeTaken-timeEstimatedInRAInMS;
                        this->startRATracking();
                        ui->pbStopTracking->setDisabled(true);
                    }
                }
            }
                if (this->mountMotion.emergencyStopTriggered==true) {
                    this->mountMotion.emergencyStopTriggered=false;
                    return;
                }
            this->mountMotion.GoToIsActiveInDecl=false;
        }
    }
    usleep(100);

    this->stopRATracking();
    if (abs(timeDifference)>100) {
        corrsteps=(0.0041780746*((double)(timeDifference))/1000.0)*
                   convertDegreesToMicrostepsRA;
        futureStepperBehaviourRA_Corr = QtConcurrent::run(this->StepperDriveRA,
                &QStepperPhidgetsRA::travelForNSteps,corrsteps, 1,10);
        while (!futureStepperBehaviourRA_Corr.isFinished()) {
           QCoreApplication::processEvents(QEventLoop::AllEvents, timeForProcessingEventQueue);
        }
        if (this->mountMotion.emergencyStopTriggered==true) {
            this->mountMotion.emergencyStopTriggered=false;
            return;
        }
    }
    this->ra=targetRA;
    this->decl=targetDecl;
    this->syncMount();
    ui->lcdGotoTime->display(0);
    ui->pbGoTo->setEnabled(true);
    ui->pbStopTracking->setDisabled(false);
    this->setControlsForGoto(true);
    this->setControlsForRATravel(true);
    return;
}
