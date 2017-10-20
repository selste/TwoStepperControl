#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QElapsedTimer>

//-------------------------------------------------------------------------------------

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    this->spiDrOnChan0 = new SPI_Drive(0);
    if (this->spiDrOnChan0->spidrGetFD() != -1) {
        this->commParams.chan0IsOpen = true;
        ui->cbspi0Open->setChecked(true);
    } else {
       this->commParams.chan0IsOpen = false;
    }
    this->spiDrOnChan1 = new SPI_Drive(1);
    if (this->spiDrOnChan1->spidrGetFD() != -1) {
        this->commParams.chan1IsOpen = true;
        ui->cbspi1Open->setChecked(true);
    } else {
       this->commParams.chan1IsOpen = false;
    }
    if (ui->rbSPI0->isChecked()) {
        this->commParams.selectedChannel = 0;
    } else {
        this->commParams.selectedChannel = 1;
    }
    this->timer = new QTimer();
    this->timer->setInterval(1000);
    this->timer->start();
    connect(ui->pbSendKinematics, SIGNAL(clicked()), this, SLOT(sendKinematics()));
    connect(ui->pbStart, SIGNAL(clicked()), this, SLOT(startDrive()));
    connect(ui->pbStop, SIGNAL(clicked()), this, SLOT(stopDrive()));
    connect(ui->pbExit, SIGNAL(clicked()), this, SLOT(terminateProgram()));
    connect(ui->rbSPI0, SIGNAL(released()), this, SLOT(switchChannel()));
    connect(ui->rbSPI1, SIGNAL(released()), this, SLOT(switchChannel()));
    connect(this->timer, SIGNAL(timeout()), this, SLOT(checkDrivesForActivity()));
    this->commParams.guiData = new QString("t"); // query type of driver ... to be completed
}

//--------------------------------------------------------------------------------------

MainWindow::~MainWindow() {
    delete ui;
}

//--------------------------------------------------------------------------------------
void MainWindow::waitForNMSecs(int msecs) {
    QElapsedTimer *qelap;

    qelap = new QElapsedTimer();
    qelap->start();
    do {
        QCoreApplication::processEvents(QEventLoop::AllEvents, msecs);
    } while (qelap->elapsed() < msecs);
    delete qelap;
}

//--------------------------------------------------------------------------------------
// send a frequent request whether drives are up ...

void MainWindow::checkDrivesForActivity(void) {
    this->commParams.guiData->clear();
    this->commParams.guiData->append("d0");
    this->commParams.guiData->clear();
    this->commParams.guiData->append("d1");
    if (this->commParams.selectedChannel == 0) {
        this->spiDrOnChan0->spidrReceiveCommand(*commParams.guiData);
    } else {
        this->spiDrOnChan1->spidrReceiveCommand(*commParams.guiData);
    }
    this->waitForNMSecs(10);
}

//--------------------------------------------------------------------------------------
// disables the drives and exits the program

void MainWindow::terminateProgram(void) {
    this->enableDrive(0,0);
    this->enableDrive(1,0);
    exit(0);
}

//--------------------------------------------------------------------------------------
// sets an internal variable that holds the spi channel to be used

void MainWindow::switchChannel(void) {
    if (ui->rbSPI0->isChecked()) {
        this->commParams.selectedChannel = 0;
    } else {
        this->commParams.selectedChannel = 1;
    }
}

//--------------------------------------------------------------------------------------
// assembles a command that takes the acceleration from the GUI and sends it to the controller

void MainWindow::getAcc(void) {
    int val;
    int driveAddressed;

    val = ui->sbAcc->value();
    if (ui->rbKinematics1->isChecked()) {
        driveAddressed = 0;
    } else {
        driveAddressed = 1;
    }
    this->commParams.guiData->clear();
    this->commParams.guiData->append("a");
    this->commParams.guiData->append(QString::number(driveAddressed));
    this->commParams.guiData->append(QString::number(val));
    if (this->commParams.selectedChannel == 0) {
        this->spiDrOnChan0->spidrReceiveCommand(*commParams.guiData);
    } else {
        this->spiDrOnChan1->spidrReceiveCommand(*commParams.guiData);
    }
    this->waitForNMSecs(150);
}

//--------------------------------------------------------------------------------------

void MainWindow::getVel(void) {
    int val;
    int driveAddressed;

    val = ui->sbVel->value();
    if (ui->rbKinematics1->isChecked()) {
        driveAddressed = 0;
    } else {
        driveAddressed = 1;
    }
    this->commParams.guiData->clear();
    this->commParams.guiData->append("v");
    this->commParams.guiData->append(QString::number(driveAddressed));
    this->commParams.guiData->append(QString::number(val));
    if (this->commParams.selectedChannel == 0) {
        this->spiDrOnChan0->spidrReceiveCommand(*commParams.guiData);
    } else {
        this->spiDrOnChan1->spidrReceiveCommand(*commParams.guiData);
    }
    this->waitForNMSecs(150);
}

//--------------------------------------------------------------------------------------

void MainWindow::getSteps(void) {
    int val;
    int driveAddressed;

    val = ui->sbSteps->value();
    if (ui->rbKinematics1->isChecked()) {
        driveAddressed = 0;
    } else {
        driveAddressed = 1;
    }
    this->commParams.guiData->clear();
    this->commParams.guiData->append("s");
    this->commParams.guiData->append(QString::number(driveAddressed));
    this->commParams.guiData->append(QString::number(val));
    if (this->commParams.selectedChannel == 0) {
        this->spiDrOnChan0->spidrReceiveCommand(*commParams.guiData);
    } else {
        this->spiDrOnChan1->spidrReceiveCommand(*commParams.guiData);
    }
    this->waitForNMSecs(150);
}
//--------------------------------------------------------------------------------------

void MainWindow::getMSteps(void) {
    QString *val;

    val = new QString("m ");
    if (ui->rbFullStep->isChecked()) {
        val->append("001");
    }
    if (ui->rbHalfStep->isChecked()) {
        val->append("002");
    }
    if (ui->rbQuarterStep->isChecked()) {
        val->append("004");
    }
    if (ui->rbEigthStep->isChecked()) {
        val->append("008");
    }
    if (ui->rbSixteenthStep->isChecked()) {
        val->append("016");
    }
    if (ui->rb32thStep->isChecked()) {
        val->append("032");
    }
    if (ui->rb64thStep->isChecked()) {
        val->append("064");
    }
    if (ui->rb128thStep->isChecked()) {
        val->append("128");
    }
    this->commParams.guiData->clear();
    this->commParams.guiData->append(val);
    delete val;
    if (this->commParams.selectedChannel == 0) {
        this->spiDrOnChan0->spidrReceiveCommand(*commParams.guiData);
    } else {
        this->spiDrOnChan1->spidrReceiveCommand(*commParams.guiData);
    }
    this->waitForNMSecs(150);
}

//--------------------------------------------------------------------------------------

void MainWindow::enableDrive(short which, short enable) {

    if (which < 0) {
        which = 0;
    }
    if (which > 1) {
        which = 1;
    }
    this->commParams.guiData->clear();
    this->commParams.guiData->append("e");
    this->commParams.guiData->append(QString::number(which));
    if (enable == 0) {
        this->commParams.guiData->append("0");
    } else {
        this->commParams.guiData->append("1");
    }
    if (this->commParams.selectedChannel == 0) {
        this->spiDrOnChan0->spidrReceiveCommand(*commParams.guiData);
    } else {
        this->spiDrOnChan1->spidrReceiveCommand(*commParams.guiData);
    }
    if (which == 0) {
        ui->cbEnableDrive1->setChecked(enable);
    } else {
        ui->cbEnableDrive2->setChecked(enable);
    }
    this->waitForNMSecs(150);
}

//--------------------------------------------------------------------------------------

void MainWindow::startDrive(void) {

    if (ui->rbCtrlDrive1->isChecked()) {
        this->enableDrive(0,1);
        this->commParams.guiData->clear();
        this->commParams.guiData->append("o0");
    } else {
        this->enableDrive(1,1);
        this->commParams.guiData->clear();
        this->commParams.guiData->append("o1");
    }
    if (this->commParams.selectedChannel == 0) {
        this->spiDrOnChan0->spidrReceiveCommand(*commParams.guiData);
    } else {
        this->spiDrOnChan1->spidrReceiveCommand(*commParams.guiData);
    }
    this->waitForNMSecs(150);
}

//--------------------------------------------------------------------------------------

void MainWindow::stopDrive(void) {

    this->commParams.guiData->clear();
    if (ui->rbCtrlDrive1->isChecked()) {
        this->enableDrive(0,0);
        this->commParams.guiData->clear();
        this->commParams.guiData->append("x0");
    } else {
        this->enableDrive(1,0);
        this->commParams.guiData->clear();
        this->commParams.guiData->append("x1");
    }
    if (this->commParams.selectedChannel == 0) {
        this->spiDrOnChan0->spidrReceiveCommand(*commParams.guiData);
    } else {
        this->spiDrOnChan1->spidrReceiveCommand(*commParams.guiData);
    }
    this->waitForNMSecs(150);
}

//--------------------------------------------------------------------------------------

void MainWindow::sendKinematics(void) {

    this->getAcc();
    this->getVel();
    this->getSteps();
    this->getMSteps();
}

//--------------------------------------------------------------------------------------
