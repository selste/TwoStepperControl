#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "qstepperphidgets.h"
#include <qtimer.h>
#include <QtConcurrent/qtconcurrentrun.h>
#include <math.h>
#include <unistd.h>
#include "QDisplay2D.h"
#include "tsc_globaldata.h"

TSC_GlobalData *g_AllData;

//------------------------------------------------------------------

MainWindow::MainWindow(QWidget *parent):QMainWindow(parent), ui(new Ui::MainWindow) {
    int serNo,verNo;
    double maa, mav;

    ui->setupUi(this); // making the widget

    g_AllData =new TSC_GlobalData(); // instantiate the global class with paraemters
    g_AllData->setSyncPosition(0.0, 0.0); // deploy a fake sync to the mount so that the microtimer starts ...
    g_AllData->monotonicGlobalTimer->start();

    QTimer *timer = new QTimer(this); // start the event timer ... this is NOT the microtimer for the mount
    timer->start(50); // check all 50 ms for events

    StepperDriveRA = new QStepperPhidgets(); // call the phidget interface to the board of the stepper
    serNo = StepperDriveRA->retrievePhidgetStepperData(1);
    verNo = StepperDriveRA->retrievePhidgetStepperData(2);
    futureStepperBehaviour = QtConcurrent::run(this->StepperDriveRA, &QStepperPhidgets::startTracking,0);
    // just initialize the queue for the motor thread
    this->trackingIsOn=false;

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
}

//------------------------------------------------------------------

MainWindow::~MainWindow() {

    delete StepperDriveRA;
    delete timer;
    delete ui;
    exit(0);
}

//------------------------------------------------------------------

void MainWindow::updateReadings() {
    if (futureStepperBehaviour.isFinished()) {
        if (this->trackingIsOn == true) {
            futureStepperBehaviour = QtConcurrent::run(this->StepperDriveRA, &QStepperPhidgets::startTracking,(g_AllData->getTimeSinceLastSync()));
        }
    }
    if (g_AllData->getINDIState() == true) {
        if (camera_client->newImageArrived() ==true) {
            this->updateCameraImage();
            camera_client->newImageUsedAsPixmap(); // set the "new image state to false
            if (ui->cbContinuous->isChecked()) {
                this->takeSingleCamShot();
            }
        }
    }  
}

//------------------------------------------------------------------

void MainWindow::startRATracking(void) {

    ui->pbStartTracking->setEnabled(0);
    this->trackingIsOn=true;
    ui->pbStopTracking->setEnabled(1);
}

//------------------------------------------------------------------

void MainWindow::stopRATracking(void) {

    ui->pbStartTracking->setEnabled(1);
    this->trackingIsOn=false;
    ui->pbStopTracking->setEnabled(0);
}
//------------------------------------------------------------------

void MainWindow::shutDownProgram() {
    ui->cbContinuous->setChecked(false);
    sleep(ui->sbExposureTime->value());
    camera_client->sayGoodbyeToINDIServer();
    delete StepperDriveRA;
    exit(0);
}

//------------------------------------------------------------------

void MainWindow::setMaxStepperAcc(void) {
    double val;

//    val = (double)(ui->sbMaxAccRA->value());
//    StepperDriveOne->setKinetics(val, 1);
}

//------------------------------------------------------------------

void MainWindow::setMaxStepperVel(void) {
    double val;

//    val = (double)(ui->sbMaxVelRA->value());
//    StepperDriveOne->setKinetics(val, 2);
}

//------------------------------------------------------------------

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
    }
}

//------------------------------------------------------------------

void MainWindow::takeSingleCamShot(void) {
   int exptime;

   exptime = (ui->sbExposureTime->value());
   camera_client->takeExposure(exptime);
}

//------------------------------------------------------------------
void MainWindow::updateCameraImage(void) {
    camImg=camera_client->getScaledPixmapFromCamera();
    this->camView->addBgImage(*camImg);
}

//------------------------------------------------------------------
void MainWindow::catalogChosen(QListWidgetItem* catalogName) {
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
void MainWindow::catalogObjectChosen(QListWidgetItem* catalogObject) {
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
void MainWindow::syncMount(void) {
    g_AllData->setSyncPosition(this->ra, this->decl);
    // convey right ascension and declination to the global parameters;
    // a microtimer starts ...
}

//------------------------------------------------------------------
void MainWindow::storeGearData(void) {
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
}
