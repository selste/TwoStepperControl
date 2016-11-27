#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "qstepperphidgetsRA.h"
#include "qstepperphidgetsDecl.h"
#include <qtimer.h>
#include <QtConcurrent/qtconcurrentrun.h>
#include <math.h>
#include <unistd.h>
#include "QDisplay2D.h"
#include "tsc_globaldata.h"

TSC_GlobalData *g_AllData;

//------------------------------------------------------------------
MainWindow::MainWindow(QWidget *parent):QMainWindow(parent), ui(new Ui::MainWindow)
{
    int serNo;
    double val;
    QString *textEntry;

    ui->setupUi(this); // making the widget

    g_AllData =new TSC_GlobalData(); // instantiate the global class with paraemters

    QTimer *timer = new QTimer(this); // start the event timer ... this is NOT the microtimer for the mount
    timer->start(100); // check all 100 ms for events

    if (g_AllData->getDriveID(0) == -1) { //no driver boards are assigned to drives
        StepperDriveRA = new QStepperPhidgetsRA(); // call the phidget interface to the board of the stepper
        serNo = StepperDriveRA->retrievePhidgetStepperData(1);
        g_AllData->setDriveData(0,serNo);

        StepperDriveDecl = new QStepperPhidgetsDecl(); // call the phidget interface to the board of the stepper
        serNo = StepperDriveDecl->retrievePhidgetStepperData(1);
        g_AllData->setDriveData(1,serNo);

        ui->lcdRAID->display(QString::number(g_AllData->getDriveID(0)));
        ui->lcdDeclID->display(QString::number(g_AllData->getDriveID(1)));
        this->StepperDriveDecl->setStopped(0);
    } else {
        dummyDrive = new QStepperPhidgetsRA(); // call the first phidget interface to the board of the stepper
        serNo = dummyDrive->retrievePhidgetStepperData(1);
        if (serNo != g_AllData->getDriveID(0)) { // dummy drive is NOT the designatedRA Drive
            StepperDriveRA = new QStepperPhidgetsRA(); // call the phidget interface to the board of the stepper
            serNo =StepperDriveRA->retrievePhidgetStepperData(1);
            g_AllData->setDriveData(0,serNo);
            this->StepperDriveRA->setStopped(0);
            ui->lcdRAID->display(QString::number(g_AllData->getDriveID(0)));
            delete dummyDrive; // set the other board to RA

            StepperDriveDecl = new QStepperPhidgetsDecl(); // call the phidget interface to the board of the stepper
            serNo = StepperDriveDecl->retrievePhidgetStepperData(1);
            g_AllData->setDriveData(1,serNo);
            this->StepperDriveDecl->setStopped(0);
            ui->lcdDeclID->display(QString::number(g_AllData->getDriveID(1)));
        } else {
            StepperDriveDecl = new QStepperPhidgetsDecl(); // call the phidget interface to the board of the stepper
            serNo = StepperDriveDecl->retrievePhidgetStepperData(1);
            g_AllData->setDriveData(1,serNo);
            ui->lcdDeclID->display(QString::number(g_AllData->getDriveID(1)));
            this->StepperDriveDecl->setStopped(0);
            delete dummyDrive; // set the other board to Decl

            StepperDriveRA = new QStepperPhidgetsRA(); // call the phidget interface to the board of the stepper
            serNo =StepperDriveRA->retrievePhidgetStepperData(1);
            g_AllData->setDriveData(0,serNo);
            this->StepperDriveRA->setStopped(0);
            ui->lcdRAID->display(QString::number(g_AllData->getDriveID(0)));
        }
    }

    this->mountMotion.RATrackingIsOn=false;
    this->mountMotion.RADriveIsMoving =false;
    this->mountMotion.DeclDriveIsMoving = false;
    this->mountMotion.DeclDriveDirection = 1;
    this->mountMotion.RADriveDirection = 1;
    this->mountMotion.RASpeedFactor=1;
    this->mountMotion.DeclSpeedFactor=1;
    ui->rbCorrSpeed->setChecked(true);

    g_AllData->setDriveParams(0,0,this->StepperDriveRA->getKinetics(3));
    g_AllData->setDriveParams(1,0,this->StepperDriveDecl->getKinetics(3));
    this->StepperDriveRA->setStepperParams((g_AllData->getDriveParams(0,1)),1);
    this->StepperDriveRA->setStepperParams((g_AllData->getDriveParams(0,2)),3);
    this->StepperDriveDecl->setStepperParams((g_AllData->getDriveParams(1,1)),1);
    this->StepperDriveDecl->setStepperParams((g_AllData->getDriveParams(1,2)),3);
    // take settings from the pref-file, except for the stepper speed, which is
    // calculated from  gear parameters

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
    delete textEntry;

    camera_client = new alccd5_client(); // install a camera client for guiding via INDI

    ui->listWidgetCatalog->addItem(QString("BrightStars")); // the catalogs
    ui->listWidgetCatalog->addItem(QString("Messier"));
    ui->listWidgetCatalog->addItem(QString("Caldwell"));
    ui->listWidgetCatalog->addItem(QString("NGC"));
    ui->listWidgetCatalog->addItem(QString("IC"));
    ui->listWidgetCatalog->addItem(QString("Herschel400"));

    this->objCatalog=NULL; // the topical catalogue
    this->ra = 0.0;        // the sync position - no sync for the mount was ccarried out
    this->decl = 0.0;

    this->camView = new QDisplay2D(ui->camTab,225,180); // make the clicakble scene view of 225 x 180 pixels
    this->camImg= new QPixmap(g_AllData->getCameraDisplaySize(0),g_AllData->getCameraDisplaySize(1)); // store the size of the scene view in the global parameter class

    connect(timer, SIGNAL(timeout()), this, SLOT(updateReadings())); // this is the event queue
    connect(ui->pbExit,SIGNAL(clicked()), this, SLOT(shutDownProgram()));
    connect(ui->pbConnectToServer,SIGNAL(clicked()),this, SLOT(setINDISAddrAndPort()));
    connect(ui->pbExpose, SIGNAL(clicked()), this, SLOT(takeSingleCamShot()));
    connect(ui->listWidgetCatalog,SIGNAL(itemClicked(QListWidgetItem*)),this,SLOT(catalogChosen(QListWidgetItem*)));
    connect(ui->listWidgetObject,SIGNAL(itemClicked(QListWidgetItem*)),this,SLOT(catalogObjectChosen(QListWidgetItem*)));
    connect(this->camView,SIGNAL(currentViewStatusSignal(QPointF)),this->camView,SLOT(currentViewStatusSlot(QPointF)));
    connect(ui->pbSync, SIGNAL(clicked()), this, SLOT(syncMount()));
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

    g_AllData->storeGlobalData();
    g_AllData->setSyncPosition(0.0, 0.0); // deploy a fake sync to the mount so that the microtimer starts ...
}
//------------------------------------------------------------------
MainWindow::~MainWindow()
{
    delete StepperDriveRA;
    delete timer;
    delete ui;
    exit(0);
}
//------------------------------------------------------------------
void MainWindow::updateReadings() {
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
    qDebug() << "RA:" << g_AllData->getActualScopePosition(0) << "Decl:" << g_AllData->getActualScopePosition(1);
}
//------------------------------------------------------------------
void MainWindow::startRATracking(void) {

    this->mountMotion.RASpeedFactor=1;
    this->StepperDriveRA->stopDrive();
    this->StepperDriveRA->setStopped(0);
    this->mountMotion.RATrackingIsOn = true;
    ui->pbStartTracking->setEnabled(0);
    ui->pbStopTracking->setEnabled(1);
    this->mountMotion.RAtrackingElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
    this->futureStepperBehaviourRA=QtConcurrent::run(this->StepperDriveRA, &QStepperPhidgetsRA::startTracking);
}
//------------------------------------------------------------------
void MainWindow::stopRATracking(void) {

    ui->pbStartTracking->setEnabled(1);
    ui->pbStopTracking->setEnabled(0);
    this->StepperDriveRA->setStopped(1);
    this->StepperDriveRA->stopDrive();
    while (!this->futureStepperBehaviourRA.isFinished()) {
    } // wait till the RA-tracking thread has died ...
    this->mountMotion.RATrackingIsOn = false;
}
//------------------------------------------------------------------
void MainWindow::shutDownProgram() {
    ui->cbContinuous->setChecked(false);
    sleep(ui->sbExposureTime->value());
    camera_client->sayGoodbyeToINDIServer();
    delete StepperDriveRA;
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
    catalogPath->append(QString(".csv"));

    this->objCatalog = new currentObjectCatalog(*catalogPath);
    maxObj = this->objCatalog->getNumberOfObjects();
    ui->listWidgetObject->clear();
    for (counterForObjects = 0; counterForObjects < maxObj; counterForObjects++) {
        objectName=this->objCatalog->getNamesOfObjects(counterForObjects);
        ui->listWidgetObject->addItem(QString(objectName.data()));
    }
}
//------------------------------------------------------------------
void MainWindow::catalogObjectChosen(QListWidgetItem* catalogObject)
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
    g_AllData->setSyncPosition(this->ra, this->decl);
    // convey right ascension and declination to the global parameters;
    // a microtimer starts ...
    this->startRATracking();
}
//------------------------------------------------------------------
void MainWindow::storeGearData(void)
{
    float pgra,ogra,wormra,ssra,pgdec,ogdec,wormdec,ssdec,microsteps;
    QString *leEntry;

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
        this->startRATracking();
    }

}
//--------------------------------------------------------------
void MainWindow::declinationMoveHandboxUp(void) {
    long maxDeclSteps;

    if (this->mountMotion.DeclDriveIsMoving==false){
        this->mountMotion.DeclMoveElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
        ui->pbDeclDown->setEnabled(0);
        ui->leAMaxDecl->setEnabled(false);
        ui->leCurrMaxDecl->setEnabled(false);
        this->mountMotion.DeclDriveIsMoving=true;
        maxDeclSteps=180/g_AllData->getGearData(7)*g_AllData->getGearData(8)*
                g_AllData->getGearData(4)*g_AllData->getGearData(5)*
                g_AllData->getGearData(6); // travel 180° at most
        this->mountMotion.DeclDriveDirection=1;
        futureStepperBehaviourDecl =
                QtConcurrent::run(this->StepperDriveDecl,
                &QStepperPhidgetsDecl::travelForNSteps,maxDeclSteps,this->mountMotion.DeclDriveDirection,1);
        while (!futureStepperBehaviourDecl.isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        }
    } else {
        this->mountMotion.DeclDriveIsMoving=false;
        this->StepperDriveDecl->stopDrive();
        while (!futureStepperBehaviourDecl.isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        }
        ui->leAMaxDecl->setEnabled(true);
        ui->leCurrMaxDecl->setEnabled(true);
        ui->pbDeclDown->setEnabled(1);
    }
}
//--------------------------------------------------------------
void MainWindow::declinationMoveHandboxDown(void) {
    long maxDeclSteps;

    if (this->mountMotion.DeclDriveIsMoving==false){
        this->mountMotion.DeclMoveElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
        ui->pbDeclUp->setEnabled(0);
        ui->leAMaxDecl->setEnabled(false);
        ui->leCurrMaxDecl->setEnabled(false);
        this->mountMotion.DeclDriveIsMoving=true;
        this->mountMotion.DeclDriveDirection = -1;
        maxDeclSteps=180/g_AllData->getGearData(7)*g_AllData->getGearData(8)*
                g_AllData->getGearData(4)*g_AllData->getGearData(5)*
                g_AllData->getGearData(6); // travel 180° at most
        futureStepperBehaviourDecl =
                QtConcurrent::run(this->StepperDriveDecl,
                &QStepperPhidgetsDecl::travelForNSteps,maxDeclSteps,this->mountMotion.DeclDriveDirection,1);
        while (!futureStepperBehaviourDecl.isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        }
    } else {
        this->mountMotion.DeclDriveIsMoving=false;
        this->StepperDriveDecl->stopDrive();
        while (!futureStepperBehaviourDecl.isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        }
        ui->leAMaxDecl->setEnabled(true);
        ui->leCurrMaxDecl->setEnabled(true);
        ui->pbDeclUp->setEnabled(1);
    }
}
//--------------------------------------------------------------

void MainWindow::RAMoveHandboxFwd(void) {
    long maxRASteps;

    if (this->mountMotion.RATrackingIsOn == true) {
        this->stopRATracking();
    }
    if (this->mountMotion.RADriveIsMoving ==false){
        this->mountMotion.RAMoveElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
        ui->pbRAMinus->setEnabled(0);
        ui->pbStartTracking->setEnabled(0);
        ui->pbStopTracking->setEnabled(0);
        this->StepperDriveRA->setStopped(1);
        ui->leAMaxRA->setEnabled(false);
        ui->leCurrMaxRA->setEnabled(false);
        this->mountMotion.RADriveIsMoving=true;
        maxRASteps=180/g_AllData->getGearData(3)*g_AllData->getGearData(8)*
                g_AllData->getGearData(0)*g_AllData->getGearData(1)*
                g_AllData->getGearData(2); // travel 180° at most

        this->mountMotion.RADriveDirection=1;
        this->mountMotion.RASpeedFactor=2; // forward motion means double the speed
        futureStepperBehaviourRA =
                QtConcurrent::run(this->StepperDriveRA,
                &QStepperPhidgetsRA::travelForNSteps,maxRASteps,
                                  this->mountMotion.RADriveDirection,
                                  this->mountMotion.RASpeedFactor);
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
        ui->leAMaxRA->setEnabled(true);
        ui->leCurrMaxRA->setEnabled(true);
        ui->pbRAMinus->setEnabled(1);
    }
}

//---------------------------------------------------------------------

void MainWindow::RAMoveHandboxBwd(void) {
    long maxRASteps;

    if (this->mountMotion.RATrackingIsOn == true) {
        this->stopRATracking();
    }
    if (this->mountMotion.RADriveIsMoving ==false){
        this->mountMotion.RAMoveElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
        ui->pbRAPlus->setEnabled(0);
        ui->pbStartTracking->setEnabled(0);
        ui->pbStopTracking->setEnabled(0);
        this->StepperDriveRA->setStopped(1);
        ui->leAMaxRA->setEnabled(false);
        ui->leCurrMaxRA->setEnabled(false);
        this->mountMotion.RADriveIsMoving=true;
        maxRASteps=180/g_AllData->getGearData(3)*g_AllData->getGearData(8)*
                g_AllData->getGearData(0)*g_AllData->getGearData(1)*
                g_AllData->getGearData(2); // travel 180° at most

        this->mountMotion.RADriveDirection=-1;
        this->mountMotion.RASpeedFactor=0; // forward motion means double the speed
        futureStepperBehaviourRA =
                QtConcurrent::run(this->StepperDriveRA,
                &QStepperPhidgetsRA::travelForNSteps,maxRASteps,
                                  this->mountMotion.RADriveDirection,
                                  this->mountMotion.RASpeedFactor);
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
        ui->leAMaxRA->setEnabled(true);
        ui->leCurrMaxRA->setEnabled(true);
        ui->pbRAPlus->setEnabled(1);
    }
}

//---------------------------------------------------------------------


void MainWindow::storeDriveData(void)
{
    g_AllData->storeGlobalData();
}
