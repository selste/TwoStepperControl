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
    int serNo;
    double maa, mav, RAStepperSpeed;
    QString *textEntry;

    ui->setupUi(this); // making the widget

    g_AllData =new TSC_GlobalData(); // instantiate the global class with paraemters

    this->RATrackingIsOn=false;

    QTimer *timer = new QTimer(this); // start the event timer ... this is NOT the microtimer for the mount
    timer->start(100); // check all 100 ms for events

    if (g_AllData->getDriveID(0) == -1) { //no driver boards are assigned to drives
        StepperDriveRA = new QStepperPhidgets(); // call the phidget interface to the board of the stepper
        serNo = StepperDriveRA->retrievePhidgetStepperData(1);
        g_AllData->setDriveData(0,serNo);

        StepperDriveDecl = new QStepperPhidgets(); // call the phidget interface to the board of the stepper
        serNo = StepperDriveDecl->retrievePhidgetStepperData(1);
        g_AllData->setDriveData(1,serNo);

        ui->lcdRAID->display(QString::number(g_AllData->getDriveID(0)));
        ui->lcdDeclID->display(QString::number(g_AllData->getDriveID(1)));
        this->StepperDriveDecl->setStopped(0);
    } else {
        dummyDrive = new QStepperPhidgets(); // call the first phidget interface to the board of the stepper
        serNo = dummyDrive->retrievePhidgetStepperData(1);
        if (serNo != g_AllData->getDriveID(0)) { // dummy drive is NOT the designatedRA Drive
            StepperDriveRA = new QStepperPhidgets(); // call the phidget interface to the board of the stepper
            serNo =StepperDriveRA->retrievePhidgetStepperData(1);
            g_AllData->setDriveData(0,serNo);
            this->StepperDriveRA->setStopped(0);
            ui->lcdRAID->display(QString::number(g_AllData->getDriveID(0)));
            delete dummyDrive; // set the other board to RA

            StepperDriveDecl = new QStepperPhidgets(); // call the phidget interface to the board of the stepper
            serNo = StepperDriveDecl->retrievePhidgetStepperData(1);
            g_AllData->setDriveData(1,serNo);
            this->StepperDriveDecl->setStopped(0);
            ui->lcdDeclID->display(QString::number(g_AllData->getDriveID(1)));
        } else {
            StepperDriveDecl = new QStepperPhidgets(); // call the phidget interface to the board of the stepper
            serNo = StepperDriveDecl->retrievePhidgetStepperData(1);
            g_AllData->setDriveData(1,serNo);
            ui->lcdDeclID->display(QString::number(g_AllData->getDriveID(1)));
            this->StepperDriveDecl->setStopped(0);
            delete dummyDrive; // set the other board to Decl

            StepperDriveRA = new QStepperPhidgets(); // call the phidget interface to the board of the stepper
            serNo =StepperDriveRA->retrievePhidgetStepperData(1);
            g_AllData->setDriveData(0,serNo);
            this->StepperDriveRA->setStopped(0);
            ui->lcdRAID->display(QString::number(g_AllData->getDriveID(0)));
        }
    }
    textEntry = new QString();
    RAStepperSpeed=(this->StepperDriveRA->getKinetics(3));
    ui->leVMaxRA->setText(textEntry->number(RAStepperSpeed,'f',2));
    textEntry->clear();
    ui->leAMaxRA->setText(textEntry->number(this->StepperDriveRA->getKinetics(2)));
    textEntry->clear();
    ui->leCurrMaxRA->setText(textEntry->number(this->StepperDriveRA->getKinetics(1)));
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
    connect(ui->pbDeclUp, SIGNAL(pressed()),this,SLOT(declinationMoveHandboxUp()));
    connect(ui->pbDeclUp, SIGNAL(released()),this,SLOT(declinationMoveHandboxStop()));
    connect(ui->pbDeclDown, SIGNAL(pressed()),this,SLOT(declinationMoveHandboxDown()));
    connect(ui->pbDeclDown, SIGNAL(pressed()),this,SLOT(declinationMoveHandboxStop()));

    g_AllData->storeGlobalData();
    g_AllData->setDriveSpeeds(0,this->StepperDriveRA->getKinetics(3));
    g_AllData->setDriveSpeeds(1,this->StepperDriveDecl->getKinetics(3));
    g_AllData->setSyncPosition(0.0, 0.0); // deploy a fake sync to the mount so that the microtimer starts ...
    this->startRATracking(); // start the RA Drive to keep track of the RA motion ...
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
    qint64 topicalTime;
    float relativeTravel;

    if (RATrackingIsOn == true) {
        topicalTime = g_AllData->getTimeSinceLastSync() - this->RAtrackingElapsedTimeInMS;
        this->RAtrackingElapsedTimeInMS=topicalTime;
        relativeTravel= 0.0041780742*topicalTime/(1000.0*
                        (g_AllData->getGearData(0))*(g_AllData->getGearData(1))*
                        (g_AllData->getGearData(2))); // total of RA-travel in degrees
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
    ui->pbStopTracking->setEnabled(1);
    this->StepperDriveRA->setStopped(0);
    this->RATrackingIsOn = true;
    this->RAtrackingElapsedTimeInMS = g_AllData->getTimeSinceLastSync();
    this->futureStepperBehaviourRA=QtConcurrent::run(this->StepperDriveRA, &QStepperPhidgets::startTracking);
}

//------------------------------------------------------------------
void MainWindow::stopRATracking(void) {

    RATrackingIsOn = false;
    ui->pbStartTracking->setEnabled(1);
    ui->pbStopTracking->setEnabled(0);
    this->StepperDriveRA->setStopped(1);
    this->StepperDriveRA->stopDrive();
    while (!this->futureStepperBehaviourRA.isFinished()) {
    } // wait till the RA-tracking thread has died ...
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
    if (this->StepperDriveRA->getStopped() == false) {
        this->stopRATracking();
    }
    g_AllData->setSyncPosition(this->ra, this->decl);
    // convey right ascension and declination to the global parameters;
    // a microtimer starts ...
    this->startRATracking();
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
    g_AllData->storeGlobalData();
    if (this->StepperDriveRA->getStopped() == false) {
        this->stopRATracking();
    }
    this->startRATracking();
}

//--------------------------------------------------------------
void MainWindow::declinationMoveHandboxUp(void) {
    long maxDeclSteps;

    ui->pbDeclDown->setEnabled(0);

    maxDeclSteps=180/g_AllData->getGearData(7)*g_AllData->getGearData(8)*
            g_AllData->getGearData(4)*g_AllData->getGearData(5)*
            g_AllData->getGearData(6); // travel 180° at most
    futureStepperBehaviourDecl =
            QtConcurrent::run(this->StepperDriveDecl,
            &QStepperPhidgets::travelForNSteps,maxDeclSteps,1,1);
    while (!futureStepperBehaviourDecl.isFinished()) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
    }
    ui->pbDeclDown->setEnabled(1);
}

//--------------------------------------------------------------
void MainWindow::declinationMoveHandboxDown(void) {
    long maxDeclSteps;

    ui->pbDeclUp->setEnabled(0);
    maxDeclSteps=180/g_AllData->getGearData(7)*g_AllData->getGearData(8)*
            g_AllData->getGearData(4)*g_AllData->getGearData(5)*
            g_AllData->getGearData(6); // travel 180° at most
    futureStepperBehaviourDecl =
            QtConcurrent::run(this->StepperDriveDecl,
            &QStepperPhidgets::travelForNSteps,maxDeclSteps,-1,1);
    while (!futureStepperBehaviourDecl.isFinished()) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
    }
    ui->pbDeclUp->setEnabled(1);
}

//--------------------------------------------------------------
void MainWindow::declinationMoveHandboxStop(void) {
    this->StepperDriveDecl->setStopped(true);
}

//--------------------------------------------------------------
