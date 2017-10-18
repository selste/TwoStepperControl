#include "mainwindow.h"
#include "ui_mainwindow.h"

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
}

//--------------------------------------------------------------------------------------

MainWindow::~MainWindow() {
    delete ui;
}

//--------------------------------------------------------------------------------------

void MainWindow::enableDrive(void) {

}
