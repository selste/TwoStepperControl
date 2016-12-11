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
    this->lx200IsOn = false;
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
    // take settings from the pref-file, except for the stepper speed, which is
    // calculated from  gear parameters

    camera_client = new alccd5_client(); // install a camera client for guiding via INDI

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

    this->camView = new QDisplay2D(ui->camTab,225,180); // make the clicakble scene view of 225 x 180 pixels
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

    RAdriveDirectionForNorthernHemisphere = 1; //switch this for the southern hemisphere to -1 ... RA is inverted
    g_AllData->storeGlobalData();
    g_AllData->setSyncPosition(0.0, 0.0); // deploy a fake sync to the mount so that the microtimer starts ...

    this->lx200port= new lx200_communication();
    if (this->lx200port->getPortState() == 1) {
        ui->cbRS232Open->setChecked(true);
    }
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
    qint64 topicalTime;
    double relativeTravelRA, relativeTravelDecl,totalGearRatio;

    if (g_AllData->getINDIState() == true) {
        if (camera_client->newImageArrived() ==true) {
            this->updateCameraImage();
            camera_client->newImageUsedAsPixmap(); // set the "new image state to false
            if (ui->cbContinuous->isChecked()) {
                this->takeSingleCamShot();
            }
        }
    }
    if (this->lx200IsOn) {
        if (lx200port->getPortState() == 1) {
                   // read the serial port
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
    }
    ui->leHourAngle->setText(textEntry->number(g_AllData->getActualScopePosition(0),'f',5));
    ui->leDecl->setText(textEntry->number(g_AllData->getActualScopePosition(1),'f',5));
}
//------------------------------------------------------------------
void MainWindow::startRATracking(void) {

    this->setControlsForRATravel(false);
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

    this->setControlsForRATravel(true);
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
void MainWindow::updateCameraImage(void)
{
    camImg=camera_client->getScaledPixmapFromCamera();
    this->camView->addBgImage(*camImg);
}
//------------------------------------------------------------------
void MainWindow::catalogChosen(QListWidgetItem* catalogName)
{
    QString *catalogPath;
    long counterForObjects, maxObj;
    std::string objectName;

    if (this->objCatalog != NULL) {
        delete this->objCatalog;
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
}
//------------------------------------------------------------------

void MainWindow::catalogObjectChosen(void)
{
    QString lestr;
    long indexInList;

    indexInList = ui->listWidgetObject->currentRow();
    if (this->objCatalog != NULL) {
        this->ra=this->objCatalog->getRADec(indexInList);
        this->decl=this->objCatalog->getDeclDec(indexInList);
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
        if (ui->rbMoveSpeed->isChecked()==false) {
            ui->sbMoveSpeed->setEnabled(true);
        } else {
            ui->sbMoveSpeed->setEnabled(false);
        }
    }
}
//--------------------------------------------------------------
void MainWindow::declinationMoveHandboxDown(void)
{
    long maxDeclSteps;

    if (this->mountMotion.DeclDriveIsMoving==false){
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
    if (this->mountMotion.RADriveIsMoving ==false){
        this->mountMotion.RAMoveElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
        ui->pbRAMinus->setEnabled(0);
        //this->StepperDriveRA->setStopped(1);
        ui->pbStartTracking->setEnabled(0);
        ui->pbStopTracking->setEnabled(0);
        this->setControlsForRATravel(false);
        this->mountMotion.RADriveIsMoving=true;
        maxRASteps=180/g_AllData->getGearData(3)*g_AllData->getGearData(8)*
                g_AllData->getGearData(0)*g_AllData->getGearData(1)*
                g_AllData->getGearData(2); // travel 180° at most

        this->mountMotion.RADriveDirection=1;
        fwdFactor = this->mountMotion.RASpeedFactor+1; // forward motion means increasethe speed
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
        this->startRATracking();
        ui->pbRAMinus->setEnabled(1);
        this->setControlsForRATravel(true);
        ui->rbCorrSpeed->setEnabled(true);
        ui->rbMoveSpeed->setEnabled(true);
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
    if (this->mountMotion.RADriveIsMoving ==false){
        this->mountMotion.RAMoveElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
        ui->pbRAPlus->setEnabled(0);
        //this->StepperDriveRA->setStopped(1);
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
        this->startRATracking();
        ui->pbRAPlus->setEnabled(1);
        setControlsForRATravel(true);
        ui->rbCorrSpeed->setEnabled(true);
        ui->rbMoveSpeed->setEnabled(true);
        if (ui->rbMoveSpeed->isChecked()==false) {
            ui->sbMoveSpeed->setEnabled(true);
        }
    }
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
}

//---------------------------------------------------------------------

void MainWindow::setCorrectionSpeed(void)
{
    this->mountMotion.RASpeedFactor = 1;
    this->mountMotion.DeclSpeedFactor = 1;
    ui->sbMoveSpeed->setEnabled(true);
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
    ui->LX200Tab->setEnabled(isEnabled);
}

//---------------------------------------------------------------------

void MainWindow::switchToLX200(void) {
    if (this->lx200IsOn==false) {
        this->lx200IsOn=true;
        ui->catTab->setEnabled(false);
        ui->pbLX200Active->setText("Deactivate LX200");
    } else {
        this->lx200IsOn=false;
        ui->catTab->setEnabled(true);
        ui->pbLX200Active->setText("Activate LX200");
    }
}

//---------------------------------------------------------------------

void MainWindow::startGoToObject(void)
{
    double travelRA, travelDecl, speedFactorRA, speedFactorDecl,TRamp, SRamp,
            SAtFullSpeed, TAtFullSpeed, earthTravelDuringGOTOinMSteps,
            convertDegreesToMicrostepsDecl,convertDegreesToMicrostepsRA;
    float targetRA, targetDecl;
    qint64 timestampGOTOStarted, timeDifference, timeTaken;
    qint64 timeEstimatedInRAInMS = 0;
    qint64 timeEstimatedInDeclInMS = 0;
    long int RASteps, DeclSteps,corrsteps;
    int timeForProcessingEventQueue = 100;
    bool RAtakesLonger;

    ui->pbGoTo->setEnabled(false);
    if (this->mountMotion.RADriveIsMoving == true) {
        this->mountMotion.RADriveIsMoving=false;
        this->StepperDriveRA->stopDrive();
        while (!futureStepperBehaviourRA.isFinished()) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, timeForProcessingEventQueue);
        }
    }
    if (this->mountMotion.DeclDriveIsMoving == true) {
        this->mountMotion.DeclDriveIsMoving=false;
        this->StepperDriveDecl->stopDrive();
        while (!futureStepperBehaviourDecl.isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, timeForProcessingEventQueue);
        }
    }
    if (this->mountMotion.RATrackingIsOn == true) {
        this->stopRATracking();
    }     // terminate all current motions ...

    this->setControlsForGoto(false);
    ui->pbStartTracking->setEnabled(false);

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
    DeclSteps=abs(travelDecl)*convertDegreesToMicrostepsDecl;
    convertDegreesToMicrostepsRA=1.0/g_AllData->getGearData(3)*g_AllData->getGearData(8)*
            g_AllData->getGearData(0)*g_AllData->getGearData(1)*g_AllData->getGearData(2);
    RASteps=abs(travelRA)*convertDegreesToMicrostepsRA;

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
    this->approximateGOTOSpeedRA=RASteps/(timeEstimatedInRAInMS/1000.0);
    this->approximateGOTOSpeedDecl=DeclSteps/(timeEstimatedInDeclInMS/1000.0);
    ui->lcdGotoTime->display(round(gotoETA/1000.0));
    // determined the estimated duration of the GoTo - Process
    QCoreApplication::processEvents(QEventLoop::AllEvents, timeForProcessingEventQueue);

    // let the games begin...
    bool RARideIsDone = false;
    elapsedGoToTime->start();
    futureStepperBehaviourRA_GOTO =
            QtConcurrent::run(this->StepperDriveRA,
            &QStepperPhidgetsRA::travelForNSteps,RASteps,
                              this->mountMotion.RADriveDirection,
                              speedFactorRA);
    while (!futureStepperBehaviourRA.isStarted()) {
    }
    this->mountMotion.GoToIsActiveInRA=true;
    this->mountMotion.RAGoToElapsedTimeInMS=g_AllData->getTimeSinceLastSync();

    timestampGOTOStarted = g_AllData->getTimeSinceLastSync();
    futureStepperBehaviourDecl_GOTO =
            QtConcurrent::run(this->StepperDriveDecl,
            &QStepperPhidgetsDecl::travelForNSteps,DeclSteps,
                              this->mountMotion.DeclDriveDirection,
                              speedFactorDecl);
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
                if (RARideIsDone==false) {
                    RARideIsDone=true;
                    timeTaken = g_AllData->getTimeSinceLastSync()-timestampGOTOStarted;
                    timeDifference = timeTaken-timeEstimatedInRAInMS;
                    this->startRATracking();
                    ui->pbStopTracking->setDisabled(true);
                }
            }
        }
        this->mountMotion.GoToIsActiveInDecl=false;
    }
    usleep(100);
    this->stopRATracking();
    if (abs(timeDifference)>100) {
        corrsteps=(0.0041780746*((double)(timeDifference))/1000.0)*
                   convertDegreesToMicrostepsRA;
        futureStepperBehaviourRA_Corr = QtConcurrent::run(this->StepperDriveRA,
                &QStepperPhidgetsRA::travelForNSteps,corrsteps, 1,10);
        while (!futureStepperBehaviourDecl.isStarted()) {
        }
        while (!futureStepperBehaviourRA_Corr.isFinished()) {
           QCoreApplication::processEvents(QEventLoop::AllEvents, timeForProcessingEventQueue);
        }
    }
    this->ra=targetRA;
    this->decl=targetDecl;
    this->syncMount();
    ui->lcdGotoTime->display(0);
    ui->pbGoTo->setEnabled(true);
    ui->pbStopTracking->setDisabled(false);
    this->setControlsForGoto(true);
    this->setControlsForRATravel(false);
}

//----------------------------------------------------------

