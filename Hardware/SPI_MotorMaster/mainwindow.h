#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "spi_drive.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    SPI_Drive *spiDrOnChan0;
    SPI_Drive *spiDrOnChan1;
    struct currentCommunicationParameters {
        bool chan0IsOpen;
        bool chan1IsOpen;
        int selectedChannel;
    };
    currentCommunicationParameters commParams;

private slots:
    void enableDrive(void);
};

#endif // MAINWINDOW_H
