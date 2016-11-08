#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "qstepperphidgets.h"
#include <qtimer.h>
#include <QtConcurrent/qtconcurrentrun.h>
#include <math.h>
#include <unistd.h>

//------------------------------------------------------------------

MainWindow::MainWindow(QWidget *parent):QMainWindow(parent), ui(new Ui::MainWindow) {
    int serNo,verNo, encoderNum;
    double maa, mav;

    ui->setupUi(this);

    QTimer *timer = new QTimer(this);
    timer->start(100);

    EncoderDriveOne = new QEncoderPhidgets();
    encoderNum = EncoderDriveOne->retrievePhidgetEncoderData(3);

    StepperDriveOne = new QStepperPhidgets();
    serNo = StepperDriveOne->retrievePhidgetStepperData(1);
    verNo = StepperDriveOne->retrievePhidgetStepperData(2);
    maa = StepperDriveOne->getKinetics(2);
    ui->sbSteps->setValue(10000);
    ui->sbMaxAccRA->setValue(maa);
    ui->sbMaxAccRA->setMaximum(maa);
    ui->sbMaxAccRA->setMinimum(maa*0.0001);
    mav=StepperDriveOne->getKinetics(3);
    ui->sbMaxVelRA->setMaximum(mav);
    ui->sbMaxVelRA->setMinimum(StepperDriveOne->getKinetics(4));
    ui->sbMaxVelRA->setValue(mav);

    camera_client = new alccd5_client();

    connect(timer, SIGNAL(timeout()), this, SLOT(updateReadings()));
    connect(ui->pushButtonGo, SIGNAL(clicked()), this, SLOT(executeSteps()));
    connect(ui->sbMaxAccRA, SIGNAL(valueChanged(int)), this, SLOT(setMaxStepperAcc()));
    connect(ui->sbMaxVelRA, SIGNAL(valueChanged(int)), this, SLOT(setMaxStepperVel()));
    connect(ui->pbExit,SIGNAL(clicked()), this, SLOT(shutDownProgram()));
    connect(ui->pbConnectToServer,SIGNAL(clicked()),this, SLOT(setINDISAddrAndPort()));
    connect(ui->pbExpose, SIGNAL(clicked()), this, SLOT(takeSingleCamShot()));
}

//------------------------------------------------------------------

MainWindow::~MainWindow() {

    delete StepperDriveOne;
    delete EncoderDriveOne;
    delete timer;
    delete ui;
    exit(0);
}

//------------------------------------------------------------------

void MainWindow::updateReadings() {
    float encoderReading;

    encoderReading=round(EncoderDriveOne->getTopicalReadingFromEncoder()*EncoderDriveOne->getCalibrationFactor());
    ui->lcdEncRA->display(encoderReading);
    if (camera_client->newImageArrived() ==true) {
        this->updateCameraImage();
        camera_client->newImageUsedAsPixmap(); // set the "new image state to false
        this->takeSingleCamShot();
    }
}

//------------------------------------------------------------------

void MainWindow::executeSteps() {
    int stepsSet;
    float encoderReading;

    ui->pushButtonGo->setEnabled(0);
    EncoderDriveOne->resetEncoder();
    encoderReading=round(EncoderDriveOne->getTopicalReadingFromEncoder()*EncoderDriveOne->getCalibrationFactor());
    ui->lcdEncRA->display(encoderReading);
    QCoreApplication::processEvents(QEventLoop::AllEvents, 500);
    stepsSet=ui->sbSteps->value();

    futureStepperBehaviour = QtConcurrent::run(this->StepperDriveOne, &QStepperPhidgets::sendSteps,(round(stepsSet)));
    while (!futureStepperBehaviour.isFinished()) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
    }
    ui->pushButtonGo->setEnabled(1);

}

//------------------------------------------------------------------

void MainWindow::shutDownProgram() {
    this->StepperDriveOne->shutDownDrive();
    delete StepperDriveOne;
    delete EncoderDriveOne;
    exit(0);
}

//------------------------------------------------------------------

void MainWindow::setMaxStepperAcc(void) {
    double val;

    val = (double)(ui->sbMaxAccRA->value());
    StepperDriveOne->setKinetics(val, 1);
}

//------------------------------------------------------------------

void MainWindow::setMaxStepperVel(void) {
    double val;

    val = (double)(ui->sbMaxVelRA->value());
    StepperDriveOne->setKinetics(val, 2);
}

//------------------------------------------------------------------

void MainWindow::setINDISAddrAndPort(void) {
    QString saddr;
    int sport;
    bool isServerUp = 0;

    saddr=ui->leINDIServer->text();
    sport=ui->sbINDIPort->value();
    isServerUp = camera_client->setINDIServer(saddr,sport);
    qDebug() << isServerUp;
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
    ui->camLabel->setPixmap(*camImg);
    ui->camLabel->show();
}
